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
CPU and GPU modes use the same common CFL timestep and retain ordered port
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

## Build and checks

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
These establish implementation agreement, not agreement with a measured engine.

The selected CPU regression run passed 53 checks, with the CUDA-only comparison
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
