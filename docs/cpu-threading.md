# CPU fluid-loop threading (`ENGINE_SIM_THREADS`)

Status: implemented and verified 2026-09-28 (Phase 3 of the optimization work).

The CPU simulation is single-threaded by default. `ENGINE_SIM_THREADS` opts into
a fixed worker pool that runs the fluid loop's commutable phases concurrently.
Results are bit-identical to the single-threaded path; only wall time changes.

```
ENGINE_SIM_THREADS=8 ./engine-sim-headless test/scripts/hayabusa.mr es 0.25 0.25 0.2
```

* Unset or `1` (default): every region runs inline on the caller — the original
  serial code with one branch. The GPU-coupled path never touches the pool.
* Counts above the physical-core count are clamped (`GetLogicalProcessorInformation`,
  one entry per core): on a 16-logical/8-physical machine, `16` runs as `8`.
  SMT siblings oversubscribe the barrier and made t16 ~3x *slower* than serial
  before the clamp; use roughly physical cores.
* The pool prints `[sim] ENGINE_SIM_THREADS=...` once on first use.

## Regions and why they stay bit-identical

Each index runs exactly the work one serial iteration would run; indices write
disjoint state; reductions are folded by the caller in index order.

| Region | Units | Disjointness argument |
| --- | --- | --- |
| reservoirs + CFL (merged) | exhaust systems, intakes, pipes | reservoir systems/atmospheres vs pipe cells never overlap; reservoir outputs are only read by the later port phase |
| ports (merged) | intake chain, exhaust chain, one unit per cylinder | chains touch plenums/collectors + opposite pipe ends; cylinder stages touch the chamber and the *other* pipe end; per-chamber pipes are distinct objects |
| pipes | one per pipe | each pipe owns its cells |

Guards:

* The merged port region requires **every** pipe to have at least 2 cells
  (`m_portsRegionMerged`, computed in `loadSimulation`). Single-cell pipes would
  alias the chain and cylinder endpoints; such builds fall back to two
  consecutive port regions (and engines with inactive pipes keep the old
  `flowPorts` path).
* Chain order inside each reservoir is preserved (a unit runs the whole cylinder
  loop serially): plenums/collectors are shared across cylinders, so reordering
  would change results.
* CFL: per-pipe `stableTimestep()` runs concurrently, then the minimum is folded
  in pipe-index order — the same order as the original loop.
* Exceptions from workers (e.g. positivity errors) are captured and rethrown on
  the calling thread after the barrier instead of terminating the process.

## Barrier design (the expensive lesson)

Regions are tiny (a few microseconds), so there are 120k+ barriers per 0.25 s
run and barrier cost dominates everything else. Two lessons:

1. **Merge regions before tuning the barrier.** Reservoirs used to be their own
   region per fluid step; they are now disjoint units of the first substep's
   CFL region. The two port regions merged into one. Fewer barriers beat a
   cheaper barrier. (With profiling enabled, note that the `cfl` bucket now
   includes reservoir time on pipe-connected engines; the `reservoirs` bucket
   only reports the no-pipe path.)
2. **Never yield-loop in a barrier under background load.** The first design
   spun 4096 pauses and then fell back to `std::this_thread::yield()` until the
   last slice arrived. Yield-looping threads stay runnable and get cycled
   through foreign tasks (this machine runs python/Unreal/Taskmgr in the
   background), so barrier latency collapsed into timer-tick scale — one run
   took 697 s where the quiet-machine time is 0.44 s, and t8 became *slower*
   than t1. The barrier now spins a bounded pause budget and then **parks on a
   condition variable**; the last arriving thread notifies only if waiters are
   parked (two-phase check, so no wakeup can be lost). Under load, waits are
   bounded by one wakeup; on a quiet machine the spin window covers every wait.

## Verification

* **Metric equality**: `ENGINE_SIM_THREADS` unset vs `8` prints identical
  `rpm`, `peak_rpm`, `samples`, `wall_temperature_K`, `coolant_energy_J`,
  `oil_temperature_K`, `piston_friction_energy_J` on both engines (and
  `rms`/`clipped` whenever the pre-existing audio-thread jitter happens to land
  on the same sample). `rms`/`clipped` differ run-to-run even between two
  single-threaded runs (pre-existing audio race) and are not used as an oracle.
* `engine-sim-test.exe` on the CPU build: 64/64 sim tests pass (58 ran, 6
  CUDA-gated skipped, 3 disabled diagnostics); the full ctest tree still fails
  only the piranha/CsvData/Optimization dependency suites (pre-existing, those
  binaries do not link this code).
* CUDA build: 64/64 with `ENGINE_SIM_GPU=1`.

## Benchmarks

`build/phase3-cpu-threads.ps1` (local; `build/` is gitignored) rotates configs
across 3 runs per script, CPU backend, 0.25 s simulated, background load
present. Medians, wall seconds:

| Script | t1 | t4 | t8 | speedup (t8 vs t1) |
| --- | ---: | ---: | ---: | ---: |
| hayabusa | 0.876 | 0.449 | **0.443** | 1.98x |
| ferrari_v12 | 1.208 | 0.565 | **0.458** | 2.64x |

Phase 0 quiet-machine baselines were 0.840 / 1.237, so the threaded path is
~1.9x / ~2.7x faster than the original single-threaded CPU path. The V12
(24 pipes, 12 cylinders) scales better than the Hayabusa (8 pipes, 4
cylinders); on the Hayabusa t4 already matches t8 because the port chains are
short and barrier cost grows with thread count.

The threaded CPU path now runs at roughly 0.57x real time (0.253 s simulated
in 0.443 s) and, on this machine, beats the coupled-GPU path for the same
Hayabusa case by a wide margin (17.7 s with `ENGINE_SIM_GPU=1
ENGINE_SIM_GPU_COUPLED=1`).

## What is left on the table

* The single-plenum intake chain is an inherent serial chain inside the port
  region (all cylinders share one plenum), so the ports region barely shrinks
  with thread count.
* Chamber `ignite`/`update` and `aggregatePipes` (a few percent of the step)
  still run serially.
* Barrier cost is still ~1-2 us per region; halving it again would require
  coarser region batching across substeps, which changes phase semantics.
