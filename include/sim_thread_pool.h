#ifndef ENGINE_SIM_SIM_THREAD_POOL_H
#define ENGINE_SIM_SIM_THREAD_POOL_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

// Opt-in worker pool for the CPU fluid loop.
//
// ENGINE_SIM_THREADS selects the total thread count (default 1). A value of
// 1 or unset leaves the simulation single-threaded: parallelFor then runs the
// loop inline on the caller, so the disabled path is the original code with
// one branch. Workers hot-spin across the short gaps between regions and park
// on a condition variable during long serial phases (mechanical solve), so
// enabled workers cost almost nothing while waiting for the next region.
//
// Region rules (the reason results stay bit-identical):
//  - each index runs exactly the work one serial iteration would run;
//  - indices write disjoint state (no writes are reduced across threads);
//  - any reduction (CFL) is folded in index order by the caller afterwards.
namespace sim_pool {

class Pool {
    public:
        static Pool &instance() {
            static Pool pool;
            return pool;
        }

        int size() const { return m_threadCount; }

        template <class F>
        void run(int count, bool pull, const F &fn) {
            m_count = count;
            m_context = &fn;
            m_call = [](const void *context, int i) {
                (*static_cast<const F *>(context))(i);
            };
            m_next.store(0, std::memory_order_relaxed);
            m_pull.store(pull, std::memory_order_relaxed);
            m_epoch.fetch_add(1, std::memory_order_release);
            {
                // Lock/unlock pairs the epoch store with parked workers so a
                // notify cannot slip between their predicate check and wait.
                std::lock_guard<std::mutex> lock(m_mutex);
            }
            m_wake.notify_all();

            // Exceptions (e.g. positivity errors from the serial physics) must
            // not escape a worker thread (that would terminate); the first one
            // wins and is rethrown on the caller after the barrier.
            try {
                if (pull) {
                    // Dynamic index pull for heterogeneous units (a port chain
                    // or a reservoir can dominate a static slice): each thread
                    // takes the next index, so arrival skew collapses while
                    // per-index work stays unchanged and disjoint. On balanced
                    // regions the contended counter costs more than it saves,
                    // so those use static slices.
                    for (;;) {
                        const int i = m_next.fetch_add(1, std::memory_order_relaxed);
                        if (i >= count) break;
                        fn(i);
                    }
                } else {
                    const int lo = slice(0, count);
                    const int hi = slice(1, count);
                    for (int i = lo; i < hi; ++i) fn(i);
                }
            } catch (...) {
                captureException();
            }
            arriveAndWait();
            std::exception_ptr error = takeException();
            if (error) std::rethrow_exception(error);
        }

    private:
        using Call = void (*)(const void *, int);

        Pool() {
            int threads = 1;
            if (const char *env = std::getenv("ENGINE_SIM_THREADS")) {
                const int requested = std::atoi(env);
                if (requested > 1) threads = requested;
            }
            const unsigned hardware = std::thread::hardware_concurrency();
            if (hardware > 0 && threads > static_cast<int>(hardware)) {
                threads = static_cast<int>(hardware);
            }
            // SMT siblings oversubscribe the barrier: retested 2026-09-28 with
            // the spin-then-park barrier and dynamic pull (t12/t16 on 8 physical
            // cores) and still slower than physical cores alone - t12 ~10-30%
            // worse, t16 ~5x worse (barrier wait explodes), so cap at physical.
            const int physical = physicalCoreCount();
            if (physical > 0 && threads > physical) {
                std::printf("[sim] ENGINE_SIM_THREADS clamped %d -> %d physical cores\n",
                    threads, physical);
                threads = physical;
            }
            m_threadCount = threads;
            if (threads <= 1) return;
            std::printf("[sim] ENGINE_SIM_THREADS=%d\n", threads);
#ifdef ENGINE_SIM_PROFILE
            // Per-participant barrier wait accounting (printed at exit):
            // slot 0 = caller, slots 1..T-1 = workers. Single-writer per slot.
            m_waitSlots.assign(static_cast<size_t>(threads), WaitSlot{});
            slotIndex() = 0;
#endif
            m_workers.reserve(threads - 1);
            for (int t = 1; t < threads; ++t) {
                m_workers.emplace_back(&Pool::workerLoop, this, t);
            }
        }

        ~Pool() {
            m_stop.store(true, std::memory_order_release);
            {
                std::lock_guard<std::mutex> lock(m_mutex);
            }
            m_wake.notify_all();
            for (std::thread &worker : m_workers) {
                if (worker.joinable()) worker.join();
            }
#ifdef ENGINE_SIM_PROFILE
            double sum = 0.0;
            long long calls = 0;
            for (const WaitSlot &slot : m_waitSlots) {
                sum += slot.seconds;
                calls += slot.calls;
            }
            if (calls > 0) {
                std::printf(
                    "[sim] barrier: %lld calls, wall %.6fs, avg %.2fus per call "
                    "(%lld barriers over %d threads)\n",
                    calls, sum / m_threadCount,
                    1e6 * sum / static_cast<double>(calls),
                    calls / m_threadCount, m_threadCount);
            }
#endif
        }

        Pool(const Pool &) = delete;
        Pool &operator=(const Pool &) = delete;

#ifdef ENGINE_SIM_PROFILE
        struct WaitSlot {
            double seconds = 0.0;
            long long calls = 0;
        };
        static int &slotIndex() {
            static thread_local int slot = -1;
            return slot;
        }
        struct WaitRecorder {
            WaitSlot *m_slot;
            std::chrono::steady_clock::time_point m_start;
            explicit WaitRecorder(WaitSlot *slot)
                : m_slot(slot), m_start(std::chrono::steady_clock::now()) {}
            ~WaitRecorder() {
                if (m_slot) {
                    m_slot->seconds += std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - m_start).count();
                    ++m_slot->calls;
                }
            }
        };
#endif

        static int physicalCoreCount() {
#if defined(_WIN32)
            ULONG length = 0;
            GetLogicalProcessorInformation(nullptr, &length);
            if (length == 0) return 0;
            std::vector<SYSTEM_LOGICAL_PROCESSOR_INFORMATION> info(
                length / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION) + 1);
            if (!GetLogicalProcessorInformation(info.data(), &length)) return 0;
            int cores = 0;
            const size_t count = length / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);
            for (size_t i = 0; i < count; ++i) {
                if (info[i].Relationship == RelationProcessorCore) ++cores;
            }
            return cores;
#else
            return 0;
#endif
        }

        int slice(int edge, int count) const {
            return static_cast<int>(static_cast<long long>(count) * edge / m_threadCount);
        }

        static void pause(int iterations) {
            for (int i = 0; i < iterations; ++i) {
#if defined(_MSC_VER)
                _mm_pause();
#else
                std::this_thread::yield();
#endif
            }
        }

        void arriveAndWait() {
#ifdef ENGINE_SIM_PROFILE
            const int slot = slotIndex();
            WaitRecorder recorder(
                (slot >= 0 && slot < static_cast<int>(m_waitSlots.size()))
                    ? &m_waitSlots[static_cast<size_t>(slot)]
                    : nullptr);
#endif
            const int generation = m_generation.load(std::memory_order_acquire);
            if (m_arrived.fetch_add(1, std::memory_order_acq_rel) == m_threadCount - 1) {
                m_arrived.store(0, std::memory_order_relaxed);
                m_generation.fetch_add(1, std::memory_order_release);
                // Wake parked waiters. The bump precedes the waiter count read
                // and waiters count before rechecking the generation, so a
                // waiter can neither miss this notify nor park unnoticed.
                if (m_waiters.load(std::memory_order_acquire) > 0) {
                    {
                        std::lock_guard<std::mutex> lock(m_barrierMutex);
                    }
                    m_barrierWake.notify_all();
                }
                return;
            }
            // Slices finish within a few microseconds when the machine is
            // quiet, so spin first. Under background load a yield loop stays
            // runnable and gets cycled through foreign tasks (barrier latency
            // then explodes); parking instead keeps the wait bounded by one
            // wakeup.
            for (int i = 0; i < 2048; ++i) {
                if (m_generation.load(std::memory_order_acquire) != generation) return;
                pause(1);
            }
            m_waiters.fetch_add(1, std::memory_order_acq_rel);
            if (m_generation.load(std::memory_order_acquire) != generation) {
                m_waiters.fetch_sub(1, std::memory_order_acq_rel);
                return;
            }
            {
                std::unique_lock<std::mutex> lock(m_barrierMutex);
                m_barrierWake.wait(lock, [this, generation] {
                    return m_generation.load(std::memory_order_acquire) != generation;
                });
            }
            m_waiters.fetch_sub(1, std::memory_order_acq_rel);
        }

        void workerLoop(int threadIndex) {
#ifdef ENGINE_SIM_PROFILE
            slotIndex() = threadIndex;
#endif
            unsigned long long seen = 0;
            for (;;) {
                int spins = 0;
                while (m_epoch.load(std::memory_order_acquire) == seen) {
                    if (m_stop.load(std::memory_order_acquire)) return;
                    if (++spins > 10000) break;
                    pause(1);
                }
                if (m_epoch.load(std::memory_order_acquire) == seen) {
                    // Long serial phase: park until run() publishes a region.
                    std::unique_lock<std::mutex> lock(m_mutex);
                    m_wake.wait(lock, [this, &seen] {
                        return m_epoch.load(std::memory_order_acquire) != seen
                            || m_stop.load(std::memory_order_acquire);
                    });
                    if (m_stop.load(std::memory_order_acquire)) return;
                }
                seen = m_epoch.load(std::memory_order_acquire);
                const int count = m_count;
                const bool pull = m_pull.load(std::memory_order_relaxed);
                const void *context = m_context;
                const Call call = m_call;
                try {
                    if (pull) {
                        for (;;) {
                            const int i = m_next.fetch_add(1, std::memory_order_relaxed);
                            if (i >= count) break;
                            call(context, i);
                        }
                    } else {
                        const int lo = slice(threadIndex, count);
                        const int hi = slice(threadIndex + 1, count);
                        for (int i = lo; i < hi; ++i) call(context, i);
                    }
                } catch (...) {
                    captureException();
                }
                arriveAndWait();
            }
        }

        void captureException() {
            std::lock_guard<std::mutex> lock(m_errorMutex);
            if (!m_error) m_error = std::current_exception();
        }

        std::exception_ptr takeException() {
            std::lock_guard<std::mutex> lock(m_errorMutex);
            std::exception_ptr error = m_error;
            m_error = nullptr;
            return error;
        }

        int m_threadCount = 1;
        std::vector<std::thread> m_workers;

        std::atomic<unsigned long long> m_epoch{0};
        std::atomic<int> m_count{0};
        std::atomic<int> m_next{0};
        std::atomic<bool> m_pull{false};
        std::atomic<const void *> m_context{nullptr};
        Call m_call = nullptr;

        std::atomic<int> m_generation{0};
        std::atomic<int> m_arrived{0};

        std::atomic<bool> m_stop{false};
        std::mutex m_mutex;
        std::condition_variable m_wake;

        std::atomic<int> m_waiters{0};
        std::mutex m_barrierMutex;
        std::condition_variable m_barrierWake;

#ifdef ENGINE_SIM_PROFILE
        std::vector<WaitSlot> m_waitSlots;
#endif

        std::mutex m_errorMutex;
        std::exception_ptr m_error;
};

// Split policy for a parallel region: Static gives each thread a fixed slice
// (best when units are the same size); Pull lets threads take indices one by
// one (best when a unit can dominate a static slice, e.g. port chains).
enum class Split { Static, Pull };

template <class F>
inline void parallelFor(int count, const F &fn) {
    Pool &pool = Pool::instance();
    if (pool.size() <= 1 || count <= 1) {
        for (int i = 0; i < count; ++i) fn(i);
        return;
    }
    pool.run(count, false, fn);
}

template <class F>
inline void parallelFor(int count, Split split, const F &fn) {
    Pool &pool = Pool::instance();
    if (pool.size() <= 1 || count <= 1) {
        for (int i = 0; i < count; ++i) fn(i);
        return;
    }
    pool.run(count, split == Split::Pull, fn);
}

} // namespace sim_pool

#endif /* ENGINE_SIM_SIM_THREAD_POOL_H */
