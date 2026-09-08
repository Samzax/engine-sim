# Experimental CUDA pipe backend

This build can execute the distributed intake/exhaust pipe interiors on an
NVIDIA GPU. It is an experimental backend, **not a demonstrated real-time speedup**.
The ordinary executable defaults to CPU physics. Use `launch-gpu.cmd` to opt in
or `launch-cpu.cmd` for the CPU path. The window title identifies the selection.
The CUDA runtime DLL is bundled. No CUDA toolkit is needed on the destination PC; a compatible NVIDIA driver is
required. The local package targets Ampere compute capability 8.6, including the
RTX 3090, and includes PTX for compatible newer devices. Older architectures
require a build targeting their compute capability. AMD/Intel GPUs are not supported.

For headless comparisons, set `ENGINE_SIM_GPU=1` or `ENGINE_SIM_GPU=0` before
running the same executable with identical script, duration, starter and throttle
arguments. The headless output identifies the selected device. Legacy lumped pipes
and constant-property gases still use the CPU. An explicitly requested GPU that
cannot initialize reports an error; it does not silently pretend to use CUDA.

## What is accelerated

The device evaluates mixture thermodynamics, Rusanov fluxes, conservative species,
momentum and total-energy updates, CFL substeps, and Darcy friction for every
distributed pipe cell. All calculations remain double precision, with the same
NASA coefficient table and temperature inversion tolerance as the CPU. No fast
math or lower-precision physics is enabled.

The engine submits all pipe interiors in a single batch per coupled fluid step.
Each mode uses a common CFL timestep across the pipes and retains ordered port
exchanges for shared plenums and collectors. This differs from the previous
per-cylinder substep schedule when different pipes impose different CFL limits.
Chambers, port exchanges, rigid-body mechanics, combustion, oil, wall heat,
controls and audio remain on the CPU. This is not a fully GPU-resident engine.

Mapped pinned buffers and a single-node CUDA graph are reused. Each pipe uses one
block and each cell one lane. Cells are read once into GPU shared memory, evolved
there, and written back once. Separate upload, status-reset and download commands
are no longer submitted. Each block records its own status; the CPU accesses
results only after stream completion. This follows NVIDIA's
[mapped-memory synchronization requirements](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html).
A reload changing the pipe count recreates the buffers and graph. Failures are
reported before results are applied to the host engine state. CPU staging copies
only active cells and reuses records rather than zeroing 64 slots on every call.
Runner summaries for audio/readouts are calculated once per mechanical step;
all physical cells still advance on every fluid step.

When every cylinder has distributed intake and exhaust pipes, port work now has
two CPU stages. First, reservoir-to-pipe exchanges run in cylinder order for
shared plenums and collectors. Then each cylinder performs its heat transfer,
blowby, valve exchange, velocity update and combustion. These stages touch
distinct ends of each pipe; pipe interiors advance after both stages. Engines
with any lumped pipe retain the original interleaved order. This separation is
preparation for moving more coupled work onto the GPU, not an additional GPU
backend or a demonstrated speedup by itself.

`SimulatorRegression.SeparatedPortsPreserveSharedReservoirAndCylinderEvolution`
compares the original and separated ordering for Hayabusa and V12 engines with
2, 8 and 64 cells per pipe. Across 20 substeps with pressure/composition gradients
and ignition, it checks exact gas-state, thermal, flame-progress and flow-total
agreement, including shared reservoirs matched by their cylinder connections.
It passes with both CPU and CUDA pipe interiors. Three alternating full-engine
runs per build/backend retained printed physical metrics; median runtime changes
were within 1.4%. A separate lumped-pipe smoke check retained printed physical
metrics. These checks validate the reordering, not real-time performance.

The GPU also returns a conservative bound for the next coupling timestep. Global
lower bounds on each species' heat capacity give an upper bound on sound speed
without repeating temperature inversion on the CPU. The solver still computes
the full NASA thermodynamics for its internal fluxes and substeps. The host
reuses this bound only while every cell's physical state is unchanged; retained
mutable references cannot leave a stale result in use. Molecular mass is cached
separately so restoring GPU state does not evaluate unused heat-capacity curves.
The bound can make coupling steps smaller than the CPU's exact acoustic limit,
so CPU and GPU trajectories need not be identical.

## Performance limits

The current CPU coupling requires a GPU round trip for every fluid step. The
bundled Hayabusa makes 160,000 such steps per simulated second. A short Nsight
Systems trace found most CUDA API time in stream synchronization and graph
submission. That trace did not capture individual graph kernel timings, so it
does not quantify the kernel's share of the total cost.

With the competing GPU workload gone, the initial backend's 0.1-second Hayabusa run took
2.42 seconds on the RTX 3090 versus 0.40 seconds on the Ryzen 7 5700X CPU path.
These are short startup observations, not universal GPU rankings. Earlier runs
made while another application saturated the GPU are excluded from performance
claims. More device-resident coupling is necessary before promising a useful
interactive acceleration. Faster hardware alone does not remove these round trips.

The subsequent mapped-memory optimization was compared with packaged revision
`bf536870`, alternating three runs of each build on the same RTX 3090. Each run
requested 0.25 simulated seconds with the starter held throughout and a 0.2 speed
input, including WAV output. Median wall times were:

| Engine | Previous GPU backend | Optimized GPU backend | Speedup |
| --- | ---: | ---: | ---: |
| Hayabusa | 5.669 s | 2.284 s | 2.48x |
| Ferrari V12 | 4.183 s | 2.144 s | 1.95x |

Reported engine metrics matched in all six before/after comparisons. A separate
check of deferred runner summaries produced byte-identical WAV output. Startup
audio varied between some repeated runs of both builds, so byte identity is not
claimed for every benchmark run. These short headless results demonstrate a GPU
backend improvement, **not real-time performance** or a GPU advantage over CPU.

A subsequent comparison of the cached CFL bound against `bc7f33f8` alternated
three runs per build, requesting 0.25 simulated seconds with the starter held
throughout and a 0.2 speed input. The GUI was closed; ordinary desktop GPU
activity fluctuated, so these are provisional short-run measurements:

| Engine | `bc7f33f8` | Cached CFL bound | Reduction in wall time |
| --- | ---: | ---: | ---: |
| Hayabusa | 2.387 s | 2.256 s | 5.5% |
| Ferrari V12 | 2.102 s | 1.799 s | 14.4% |

A single 64-cell Hayabusa pair requesting 0.1 simulated seconds took 6.817 s
before and 4.959 s after. This is a stress check, not a statistically established
speedup. Hayabusa's reported physical metrics matched at printed precision in the
default-cell runs. V12 final RPM changed from 521.476 to 521.553, and coolant
energy from 0.0285812 J to 0.029385 J, consistent with the changed coupling steps;
the runs do not establish trajectory identity or improved measured accuracy.
An additional CPU temperature-inversion cache was tested and removed because
full-engine timings did not demonstrate a reliable benefit.

### Bounded-worker transport experiment

`tools/cuda-handshake-benchmark.cu` isolates host/device transport from the
physics. It compares the existing graph-launch/synchronize pattern with workers
that accept at most 128 requests per launch. Workers also retire on a GPU cycle
budget; an idle-host check verifies retirement. CPU/GPU signaling uses aligned
32-bit system-scope acquire/release loads and stores, permitted for mapped memory
by NVIDIA's [CUDA memory model](https://docs.nvidia.com/cuda/cuda-programming-guide/05-appendices/cuda-cpp-memory-model.html),
even when the device reports no native host atomic read-modify-write support.
Every active output value is checked for all 4,096 requests per configuration.

One RTX 3090 run with eight active cells per pipe measured 24.579 versus 14.167
microseconds per request for eight pipes, and 39.517 versus 33.331 microseconds
for 24 pipes. These numbers include host payload preparation and verification;
they are **not engine timings**. Earlier 64-cell experiments showed that workers
can be slower at larger payloads. The prototype is not wired into the simulator:
it does not yet recover a partially completed request when some workers retire.
The experiment supports investigating launch amortization, but does not establish
that persistent workers alone can deliver real-time simulation.

For a Visual Studio x64 developer shell with the CUDA `bin` directory on PATH:

```powershell
nvcc -std=c++17 -arch=sm_86 -O3 --cudart shared -Xcompiler /MD -o build/cuda-handshake-benchmark.exe tools/cuda-handshake-benchmark.cu
./build/cuda-handshake-benchmark.exe 8
./build/cuda-handshake-benchmark.exe 64
```

The probe requires mapped memory and at least 24 multiprocessors. Its cycle
budget is not a precise wall-time guarantee under changing GPU clocks. A host
wait timeout or worker retirement during a request reports a failed experiment;
it must not be interpreted as a valid timing result.

## Build and checks

### Shared gas model for the coupled backend

The CUDA translation unit now compiles the existing `GasSystem` numerical source
for device execution, including mixture thermodynamics and finite-volume and
fixed-environment transfers. CPU compilation retains the same source. Device
coefficients use constant memory; double precision and `--fmad=false` remain in
effect. The added relaxed-constexpr compiler option permits constexpr helpers
across execution spaces; it does not enable fast math.

`gpu_gas::advance` is a standalone validation batch. It is not called by the live
engine yet and provides no live simulation speedup. It copies trivially copyable
gas values, runs independent transfers, and publishes results only after CUDA
completion. Moving the coupled fluid loop still requires chamber heat, combustion,
reservoir ordering, adaptive timesteps, and counters to execute consistently on
the device.

`GasSystemTests.CudaGasTransfersMatchCpuAndConserveClosedSystem` compares 660
cases over five successive transfers, including vacuum cells, different mixtures,
150–7000 K, closed ports, zero timesteps, and both gas-property modes for finite
volumes. Fixed-environment cases cover variable properties. It checks individual
states against CPU results and closed-system mass and energy conservation. This
test passed with NVIDIA Compute Sanitizer memcheck reporting zero errors. The
existing CUDA pipe comparison and separated-reservoir evolution regression also
passed. A fresh CPU-only build passed 56 selected checks, with the two CUDA-only
checks skipped.

The RTX 3090 runtime reports cooperative-launch support and 82 multiprocessors.
This makes a synchronized multi-block coupled kernel worth investigating; actual
kernel occupancy, correctness, and real-time performance remain unproven.

### Shared cylinder stages

`chamber_flow.h` now supplies the numerical cylinder stage to CPU execution and
CUDA: wall/coolant exchange, blowby, ordered intake/exhaust valve transfers,
velocity damping, flame extinction, burn advancement and fuel/flow counters.
`CombustionChamber` can export and import its gas, thermal and flame state as a
value packet. CPU execution uses references to its existing state, avoiding a
copy on every fluid substep. Mechanical updates, oil evolution, ignition and
dynamic flame-speed preparation retain their existing CPU ordering.

`gpu_chamber::advance` validates independent cylinder packets after reservoir
ports and before pipe interiors. This interface is still a development batch,
not the live coupled scheduler. It does not yet remove the per-fluid-step GPU
round trips. Reservoir/atmosphere updates and the adaptive pipe loop still need
to join the device scheduling path.

The cylinder comparison covers 72 combinations of gas-property mode, thermal
mode, valve/blowby configuration and 150–7000 K over 30 consecutive steps. It
checks gas, wall, coolant, flame and flow results, and verifies closed-system
mass and energy accounting, including reaction enthalpy's 298.15 K reference
and delta-n RT correction. The test initially exposed a CUDA reaction-limit
selection error: the initializer-list minimum returned zero burn. Explicit
pairwise minima fixed it without changing the CPU limits. The comparison then
passed, including under NVIDIA memcheck with zero errors.

The separated-port regression also exports real Hayabusa and V12 cylinder states
at 2, 8 and 64 pipe cells, compares the CUDA cylinder stage against CPU execution,
and exercises state import. The original exact CPU interleaved-versus-separated
comparison remains in place. These checks establish numerical implementation
agreement over the tested cases, not real-time performance or measured-engine
calibration.

A fresh CPU-only build passed 56 selected checks; three CUDA-only checks were
skipped and exercised separately in the CUDA build. Before/after Hayabusa and
V12 runs on both backends retained printed RPM, temperature, coolant and friction
outputs. Audio RMS/clipping varied in some runs, including repeat invocations of
the same executable; subsequent Hayabusa repeats matched the baseline. These
observations do not establish byte-identical audio or a speedup.

### Phase profiling and closed ports

Configure a diagnostic build with `-DENGINE_SIM_PROFILE=ON` to print inclusive
wall times and call counts for the simulation step, fluid work, reservoirs, CFL
selection, chamber/port work, pipe batches, and CUDA launch/completion waits.
Counters are local to each CPU thread and print on thread exit. Nested times
must not be added together. The option defaults to OFF, which compiles out the
timers entirely. Return it to OFF before release timing or packaging.

For a 0.253-second starter run on the RTX 3090, the profiled Hayabusa took
2.274 seconds inside simulation steps: 1.626 seconds in CUDA launch/completion
waits and 0.325 seconds in CPU chamber/port work. The V12 took 1.798 seconds,
including 1.040 seconds of CUDA waits and 0.450 seconds of chamber/port work.
Those wait figures include kernel execution, submission, synchronization and
scheduling; they are not isolated kernel timings. Even the measured CPU port
work exceeded the simulated duration. Moving only pipe interiors onto the GPU
therefore does not address the whole real-time bottleneck in these runs.

This investigation also tried unrolling coefficient assembly and avoiding a
dynamic coefficient-array address. CUDA stack use fell from 80 to zero bytes
per thread, while register use rose from 94 to 104. Three full-engine comparisons
did not demonstrate a reliable speedup, so that kernel experiment was reverted.

Closed ports now bypass transfer calculations when both gases have nonnegative
sensible energy. Temporarily depleted states retain the original energy-floor
path. Variable-property zero-flow atmosphere connections also return immediately.
No conductance, timestep, precision, or physical effect is reduced. Three
alternating runs per build, requesting 0.25 simulated seconds with the starter
held throughout and 0.2 speed input, gave these median wall times with timers off:

| Engine / backend | Before | Closed-port shortcut |
| --- | ---: | ---: |
| Hayabusa / CPU | 0.918 s | 0.881 s |
| V12 / CPU | 1.306 s | 1.216 s |
| Hayabusa / CUDA | 2.366 s | 2.329 s |
| V12 / CUDA | 1.864 s | 1.794 s |

These are short startup observations under desktop activity. Printed RPMs
matched. Small changes in thermal/energy outputs and audio remain possible.
Skipping evaluations can change the cached starting temperature used by later
numerical inversions; byte-identical full-engine trajectories are not
claimed. The closed-port regression checks isolated-gas conservation in both
property modes and retains the negative-energy recovery case.

Configure with `-DENGINE_SIM_CUDA=ON` and an appropriate
`-DCMAKE_CUDA_ARCHITECTURES=86`. With Visual Studio, if CUDA integration is not
detected, use a new build directory and pass a toolset such as
`-T "cuda=C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.9"`.
CPU-only builds leave the option off and do not require CUDA.

`GasSystemTests.CudaPipeMatchesCpuAndConservesEnergy`, enabled by
`ENGINE_SIM_GPU=1`, compares 8- and 64-cell pipe evolution against the CPU across
150–7000 K, mixed species, pressure gradients and friction, and checks total mass
and energy conservation, including an energy-depleted port endpoint. It has also
passed NVIDIA Compute Sanitizer memcheck.
The cached-bound version also passed memcheck and racecheck with no reported
errors or hazards. Its tests verify that the returned timestep does not exceed
the exact acoustic limit and that cell mutation invalidates reuse. Bernstein
coefficient bounds check the heat-capacity minima over the complete polynomial
intervals, including their endpoint extensions. Independent species-energy sums
check the CPU mixture cache across composition changes and temperature boundaries.
These establish implementation agreement, not agreement with a measured engine.

The selected CPU regression run passed 56 checks, with the CUDA-only comparison
skipped there and run separately with CUDA enabled. A CPU-only configuration
also builds without the CUDA toolkit dependency. Both Hayabusa and Ferrari V12
completed one simulated second on both backends with a half-second starter and
0.2 speed input. Those short starter settings did not establish sustained running.
The V12's final near-stall RPM differed (about 3 on CPU and 26 on GPU), despite
nearly identical peak cranking RPM. Full-engine trajectory equivalence and
interactive stability have therefore **not** been established. CPU saturation
during the longer runs makes their wall times unsuitable as GPU speedup figures.

The final packaged GUI, using the bundled CUDA runtime with the toolkit removed
from PATH, passed the isolated-desktop 120-frame diagnostic: initialization,
minimize/restore, successful reload, failed reload preserving the engine, video
recording and shutdown. FFmpeg decoded the recording without errors. The helper
timeout was extended locally to five minutes for this slow backend. This is a
functional check, not proof of smooth real-time interaction.

The added physical models retain the calibration limitations in
[physics-realism.md](physics-realism.md).
