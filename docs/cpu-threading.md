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
  SMT was retested 2026-09-28 with the spin-then-park barrier and `Pull`
  splitting: t12 was still 10-30% *slower* than t8 and t16 ~5x slower (the
  barrier wait explodes when two of our threads share a core and both spin),
  so the clamp stands. Use physical cores.
* The pool prints `[sim] ENGINE_SIM_THREADS=...` once on first use.

## Regions and why they stay bit-identical

Each index runs exactly the work one serial iteration would run; indices write
disjoint state; reductions are folded by the caller in index order.

| Region | Units | Disjointness argument |
| --- | --- | --- |
| chamber update + flow reset | one per cylinder | each unit writes only its chamber's state and reads shared functions (`Function::sampleTriangle`, head getters) const-only; it runs before any fluid region, so nothing it reads is in flight |
| reservoirs + CFL (merged) | exhaust systems, intakes, pipes | reservoir systems/atmospheres vs pipe cells never overlap; reservoir outputs are only read by the later port phase |
| ports (merged) | intake chain, exhaust chain, one unit per cylinder | chains touch plenums/collectors + opposite pipe ends; cylinder stages touch the chamber and the *other* pipe end; per-chamber pipes are distinct objects |
| pipes | one per pipe | each pipe owns its cells |
| aggregate pipes | one per cylinder | each chamber aggregates into its own two pipes and runner systems |

Ignition (`ignite()`) deliberately stays serial before the update region: it
consumes the global `rand()` in cylinder order, and reordering those draws
would change results. The rigid-body solver (`m_system->process`) stays
serial too - it is an inherently sequential impulse solve.

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
   background), so barrier latency collapsed into timer-tick scale - one run
   took 697 s where the quiet-machine time is 0.44 s, and t8 became *slower*
   than t1. The barrier now spins a bounded pause budget and then **parks on a
   condition variable**; the last arriving thread notifies only if waiters are
   parked (two-phase check, so no wakeup can be lost). Under load, waits are
   bounded by one wakeup; on a quiet machine the spin window covers every wait.
3. **Split work where units are heterogeneous, not everywhere.** A static
   slice strands the port region: both reservoir chains can land on one
   thread while the others run single cylinders. Those regions (`Split::Pull`)
   let each thread take the next index, which collapses the arrival skew. On
   *balanced* regions (pipes: N pipes over N threads) the contended counter is
   pure loss - pulling 24 indices through one atomic per V12 substep cost more
   than the skew it could ever save - so they keep static slices. The default
   is `Static`; call sites opt into `Pull` per region shape.
4. **Measure the imbalance before weighting the slices.** Static pipe and CFL
   slices now split at cumulative *cell-count* quantiles (`parallelFor(count,
   weights, fn)`), which is the right general shape for mixed pipe lengths.
   Measured on both benchmark engines it is a mathematical no-op: every pipe
   has exactly 8 cells (probe: hayabusa `8x8`, v12 `24x8`), so the weighted
   cut lands on the same boundaries as the uniform cut, and an interleaved
   step2-vs-step3 A/B (6 pairs per engine) showed no difference beyond the
   machine's +-5% noise. The remaining barrier wait is not slice skew - it is
   the serial port chain (wall floor) plus the per-region fixed cost
   (~0.1-0.2 s of the profile's 0.13-0.16 barrier wall at most); fusing the
   three fluid regions would save that fixed cost but breaks the per-region
   profile buckets for an estimated 1-2%, so it was left alone.

With profiling enabled (`ENGINE_SIM_PROFILE=ON`), the pool prints a barrier
account at exit, e.g. `[sim] barrier: 1052480 calls, wall 0.155362s, ...` -
`wall` is the estimated time the whole team spent waiting at barriers
(sum of per-thread times / threads), which is the first number to watch when
tuning region structure.

## AVX2 vectorization (`ENGINE_SIM_AVX2`)

The base fluid work (not just the overhead) was vectorized in two steps, both
opt-in via `ENGINE_SIM_AVX2=ON` (applies `/arch:AVX2` to the five CPU targets;
`engine-sim-cuda` is deliberately not flagged):

* `GasPipe::advance`: the conserved-state precomputes (velocity, pressure,
  `|v|+c`), the interface/flux combine and the cell update run 2-4 doubles at
  a time in ymm registers.
* `GasSystem::refreshProperties`: the NASA7 coefficient blend vectorizes over
  species; hoists in `flow`/`flowCylinderPorts`/`flowStep` trim redundant
  per-call work.

Exactness rules (why results stay identical): code is gated on
`__AVX2__` with `#else` fallbacks that are the original scalar code
byte-for-byte; elementwise operation order matches the scalar path exactly;
no FMA (verified via `dumpbin`: `vfmadd` count = 0); no reassociation of
divides or sums - `aggregate` and every reduction fold stay scalar.

Measured with interleaved run-by-run A/B against the scalar reference binary
(medians of pairs, t8): hayabusa 0.390-0.400 vs 0.410-0.442, v12 0.426-0.434
vs 0.447-0.454, i.e. roughly -5% wall on both engines at t8 and -8% at t1.
The third planned step (weighted slices, above) added nothing measurable.

## Verification

* **Metric equality**: `ENGINE_SIM_THREADS` unset vs `8` prints identical
  `rpm`, `peak_rpm`, `samples`, `wall_temperature_K`, `coolant_energy_J`,
  `oil_temperature_K`, `piston_friction_energy_J` on both engines (and
  `rms`/`clipped` whenever the pre-existing audio-thread jitter happens to land
  on the same sample). The pre-change and post-change binaries were run
  side-by-side and their full metric lines match exactly. `rms`/`clipped`
  differ run-to-run even between two single-threaded runs (pre-existing audio
  race - seen on the reference binary itself, and again on the SIMD binary
  before disappearing on re-run) and are not used as an oracle.
* `engine-sim-test.exe` on the CPU build: 64/64 sim tests pass (58 ran, 6
  CUDA-gated skipped, 3 disabled diagnostics); the full ctest tree still fails
  only the piranha/CsvData/Optimization dependency suites (pre-existing, those
  binaries do not link this code).
* CUDA build: 64/64 with `ENGINE_SIM_GPU=1`.
* **On this machine, cross-run comparisons are only trustworthy when
  interleaved**: a background python farm swings wall times by 20-100% between
  rounds, which repeatedly faked regressions and improvements that did not
  exist. The change that mattered - dynamic pulling - was verified by
  alternating the old and new binaries run-by-run (new won 4/5 pairs on the
  V12 and 3/5 on the Hayabusa, medians 0.450 vs 0.481 and 0.469 vs 0.568).

## Benchmarks

`build/phase3-cpu-threads.ps1` (local; `build/` is gitignored) rotates configs
across 3 runs per script, CPU backend, 0.25 s simulated, profile off, 26% load.
Medians, wall seconds:

| Script | t1 | t4 | t8 | speedup (t8 vs t1) | s per simulated second |
| --- | ---: | ---: | ---: | ---: | ---: |
| hayabusa | 0.812 | 0.393 | **0.396** | 2.05x | 1.57 |
| ferrari_v12 | 1.177 | 0.454 | **0.432** | 2.72x | 1.71 |

Phase 0 quiet-machine baselines were 0.840 / 1.237, so the threaded path is
~2.1x / ~2.9x faster than the original single-threaded CPU path (both engines
also beat their Phase-0 anchors at t1, which is measurement noise on a shared
machine, not a serial-code win). The V12 (24 pipes, 12 cylinders) scales better
than the Hayabusa (8 pipes, 4 cylinders); on the Hayabusa t4 already matches t8
because the port chains are short and barrier cost grows with thread count.

The threaded CPU path runs at ~1.57 simulated seconds per wall second on the
Hayabusa after the AVX2 steps (interleaved medians ~0.39-0.40 s for 0.253 s
simulated; the threading-only number above was 1.57 in its own session) and,
on this machine, beats the coupled-GPU path for the same case by a wide margin
(17.7 s with `ENGINE_SIM_GPU=1 ENGINE_SIM_GPU_COUPLED=1`). Cross-session wall
comparisons on this machine are meaningless; trust only the interleaved A/B
numbers in the AVX2 section.

## What is left on the table

* The single-plenum intake chain is an inherent serial chain inside the port
  region (all cylinders share one plenum), so the ports region barely shrinks
  with thread count. It is now the wall floor of the fluid loop; splitting it
  would require breaking the plenum dependency (not possible exactly).
* The rigid-body solver and the audio/aggregate code outside `simulateStep_`
  are serial (~0.05-0.09 s per run) and were left alone.
* `stableTimestep`'s Newton solve and the `GasSystem::flow` transaction are
  still scalar - the only levers left that shrink base work rather than
  overhead. `GasPipe::advance` is already vectorized. Chasing 1.0 s per
  simulated second would need those (branchy, tolerance-sensitive), and the
  plan's optimistic bucket arithmetic did not survive contact with the
  measurements: the three planned steps delivered ~5-6% wall on top of the
  threaded path, not the 30-50% the estimates suggested.
