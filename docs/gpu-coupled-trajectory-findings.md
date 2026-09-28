# Phase 1 (H1): is the strict 64-cell trajectory failure a GPU bug?

Status: evidence gathered, gating implemented, committed 2026-09-27.

## What the strict test measures

`SimulatorRegression.CoupledGpuPreservesAdaptiveFluidLoop` (now
`DISABLED_...`) compares a CPU reference against the coupled GPU path over a
**free-running** window: fluidSteps=8, four 1e-4 mechanical steps, 2/8/64 cells
per pipe, Hayabusa + Ferrari V12, tolerance 1e-7 relative (floor 1e-8), no
resets between steps.

## Measurement method (corrected)

Every comparison in `compareCoupledFluidLoop` records `|a-b| / tolerance` into
a vector when an `exceedance` output is requested. Percentiles and counts are
computed from that vector, so numbers below do not depend on parsing gtest
messages. (Earlier figures in this file were parsed from log lines and were
**wrong**: gtest wraps the failure message after "which exceeds T, where", so
the earlier regex silently dropped most messages.)

Each run below makes the same 112,640 comparisons.

| run | over-tolerance (>1x) | p99 (x tol) | max (x tol) |
| --- | ---: | ---: | ---: |
| CPU vs CPU, probe 0 (baseline) | 0 | - | - |
| CPU vs CPU, probe 1e-16 | 428 | 0.081 | 33 097 |
| CPU vs CPU, probe 1e-15 | 423 | 0.079 | 32 804 |
| CPU vs CPU, probe 1e-14 | 289 | 0.024 | 9 964 |
| CPU vs CPU, probe 1e-12 | 418 | 0.076 | 31 704 |
| **GPU vs CPU, no probe** | **418** | **0.080** | **33 706** |

Logs: `phase1-probe-stats.log` (CPU probes), `phase1-envelope-pct3.log`
(GPU), `phase1-strict-rerun.log` (assert-mode diagnostic).

Findings:

1. **Harness is exact.** Probe 0 passes: free-running CPU vs CPU with an
   identical schedule is bit-deterministic, so divergence elsewhere is a real
   state difference, not harness noise.
2. **Counts saturate, they do not grow with probe size.** 1e-16 through 1e-12
   all land in 289-428 over-tolerance comparisons out of 112,640 (0.26-0.38%),
   with maxima of 10k-33k x tolerance. Any machine-epsilon displacement
   decorrelates this fixture completely inside the 4-step window; only the
   magnitudes of the tail vary, and they do so non-monotonically because the
   trajectories re-diverge onto different states.
3. **The GPU result is inside that band.** 418 over-tolerance comparisons,
   p99 0.080, max 33,706 - statistically indistinguishable from the CPU's own
   conditioning (418 / 0.076 / 31,704 at 1e-12). All GPU divergence is in the
   Hayabusa/64 fixture: 417 assert-mode failures there plus one setup-time
   trace, and zero in every other engine/cell-count combination.
4. **Growth is exponential, not a constant offset.** Strict-test failures by
   mechanical step: step 0 = 0, step 1 = 0, step 2 = 83, step 3 = 334
   (of 112,640 comparisons each step). A systematic GPU error would start at
   nonzero; this starts at exactly zero and doubles.

### Supporting evidence already on file

- `CoupledGpuMatchesIdenticalFluidStepInputs` PASSES: same state, one fluid
  step, GPU and CPU agree inside 1e-7 on every gas/heat/flow/flame quantity,
  both engines, 2/8/64 cells.
- `CoupledSnapshotCpuControl` PASSES: stage lockstep (atmosphere / reservoir
  ports / cylinders / pipes) agrees after the reservoir-map fix.
- memcheck, racecheck, initcheck: 0 errors on the coupled kernel.
- CFL formula is identical on both sides (`gas_pipe.h:66-79` vs `gpu_pipe.cu`
  `maximum=1; fmax(|v|+c)` -> `0.25*dx/maximum`), so no structural substep
  rule difference is known.
- The documented V12 CPU/GPU metric differences (rpm 521.683 vs 521.553,
  coolant 0.0285881 vs 0.029385) come from the changed cached CFL bound, which
  changes coupling timesteps; they are documented in `docs/gpu-backend.md`.

## Conclusion

The strict failure is **the fixture's conditioning, not a GPU physics bug**:
a 1e-16 nudge on pure CPU breaks the same tolerance and reaches the same
magnitudes, and the GPU's accumulated divergence grows from exactly zero into
that same band. The 1e-7 free-running threshold measures Lyapunov amplification
in this ignition fixture, not implementation agreement.

## Gating decision (implemented)

Approved: keep the strict test as an opt-in diagnostic, promote the CPU
baseline, and add an envelope-derived bound. In `test/simulator_regression_tests.cpp`:

- `DISABLED_CoupledGpuPreservesAdaptiveFluidLoop` - the original free-running
  run, unchanged, opt in with `--gtest_also_run_disabled_tests`. Failing is
  its documented behaviour.
- `CoupledTrajectoryCpuBaseline` - probe 0, must be exactly within tolerance
  (guards harness determinism).
- `CoupledTrajectoryWithinConditioningEnvelope` - measures the GPU run and a
  CPU run displaced by 1e-12 in the same process, then asserts the GPU stays
  within 8x of the CPU envelope on three statistics: over-tolerance count,
  p99 and max. Both runs are measurement runs (recorded, not asserted), so the
  bound is self-calibrating per binary/machine. Measured headroom today:
  count 418 vs limit 3,344; p99 0.080 vs 0.80; max 33,706 vs 253,636. A
  systematic GPU error pushes nearly every comparison out of tolerance
  (~112,640 over-tolerance vs the 3,344 limit) and is caught immediately.
- `CoupledGpuMatchesIdenticalFluidStepInputs` and `CoupledSnapshotCpuControl`
  remain the step-level implementation-agreement gates.

8x was chosen over 4x because the CPU envelope itself varies with compiler and
machine; both runs share a binary, so the ratio is stable while the absolute
numbers are not.

## Reproduce

```powershell
# CPU envelope with printed statistics (opt-in diagnostic; it fails by design)
build\windows-x64\Release\engine-sim-test.exe --gtest_also_run_disabled_tests `
  --gtest_filter='*DISABLED_CoupledTrajectory*'

# Envelope gate + baseline + step-level gates
$env:ENGINE_SIM_GPU='1'
build\windows-cuda\Release\engine-sim-test.exe --gtest_filter='*Coupled*'

# Original strict failure distribution (opt-in; fails by design)
build\windows-cuda\Release\engine-sim-test.exe --gtest_also_run_disabled_tests `
  --gtest_filter='*DISABLED_CoupledGpuPreservesAdaptiveFluidLoop'
build\parse-strict.ps1 -Log <saved log>
```
