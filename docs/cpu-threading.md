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

With profiling enabled (`ENGINE_SIM_PROFILE=ON`), the pool prints a barrier
account at exit, e.g. `[sim] barrier: 1052480 calls, wall 0.155362s, ...` -
`wall` is the estimated time the whole team spent waiting at barriers
(sum of per-thread times / threads), which is the first number to watch when
tuning region structure.

## Verification

* **Metric equality**: `ENGINE_SIM_THREADS` unset vs `8` prints identical
  `rpm`, `peak_rpm`, `samples`, `wall_temperature_K`, `coolant_energy_J`,
  `oil_temperature_K`, `piston_friction_energy_J` on both engines (and
  `rms`/`clipped` whenever the pre-existing audio-thread jitter happens to land
  on the same sample). The pre-change and post-change binaries were run
  side-by-side and their full metric lines match exactly. `rms`/`clipped`
  differ run-to-run even between two single-threaded runs (pre-existing audio
  race) and are not used as an oracle.
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

The threaded CPU path now runs at 1.57 simulated seconds per wall second on
the Hayabusa (0.396 s for 0.253 s simulated) and, on this machine, beats the
coupled-GPU path for the same case by a wide margin (17.7 s with
`ENGINE_SIM_GPU=1 ENGINE_SIM_GPU_COUPLED=1`).

## What is left on the table

* The single-plenum intake chain is an inherent serial chain inside the port
  region (all cylinders share one plenum), so the ports region barely shrinks
  with thread count.
* The rigid-body solver and the audio/aggregate code outside `simulateStep_`
  are serial (~0.05-0.09 s per run) and were left alone.
* Barrier cost is still ~0.13-0.16 s of wall (the profile build's barrier
  account proves it); the remaining lever there is coarser region batching
  across substeps, which changes phase semantics.
* The cell loops themselves (`GasPipe::advance`, `stableTimestep`) are scalar;
  vectorizing them is the only lever left that shrinks the base work rather
  than the overhead, and would be needed to chase 1.0 s per simulated second.
