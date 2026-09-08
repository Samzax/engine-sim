# Headless simulation

`engine-sim-headless` runs script compilation, physics, and audio synthesis
without creating a window or opening an audio device. It prints actual simulated
time, processing wall time, final RPM, sample count, normalized audio RMS, and
the number of samples saturated at the PCM16 limits.

```powershell
./engine-sim-headless.exe assets/main.mr es 10
```

Arguments are script path, script-library directory, run duration in seconds,
optional starter duration (default 1 second), and optional throttle (default
0.1, range 0..1). Scripts without a vehicle or transmission use the application's
defaults. Relative paths are resolved from the caller's directory; absolute
paths allow running from elsewhere.

A final optional argument records the generated samples as mono 44,100 Hz PCM16:

```powershell
./engine-sim-headless.exe assets/main.mr es 10 1 0.1 subaru.wav
```

The named output file is overwritten. Recording streams to disk instead of
retaining the run in memory. A failed run may leave an incomplete output file;
only a successful exit confirms a finalized WAV. Recorded duration follows the
actual simulated time, and recording I/O contributes to reported wall time.

For a source checkout:

```powershell
cmake --build build/windows --config Release --target engine-sim-headless
./build/windows/Release/engine-sim-headless.exe test/scripts/hayabusa.mr es 10 3 0.2
./build/windows/Release/engine-sim-headless.exe test/scripts/ferrari_v12.mr es 10
```

The runner uses synchronous audio rendering and drains every frame. It retains
the simulator's adaptive frame-step logic and reports actual elapsed simulation
time, which can slightly exceed the requested duration. A nonzero exit indicates
an input/compilation error, non-finite state, no simulation progress, or no audio
samples. It does not assume a particular RPM or clipping count is a pass/fail
condition: starter duration, throttle, and engine settings affect these outcomes.
This is a diagnostic tool, not a substitute for graphics/audio-device testing.

## Local observations (Windows Release, 2026-09-08)

| Engine / controls | Simulated | Wall time | Final RPM | Clipped samples |
| --- | --- | --- | --- | --- |
| Hayabusa, 1s starter, 10% throttle | 10.007s | 6.96s | approximately 0 (stalled) | 0 / 441316 |
| Hayabusa, 3s starter, 20% throttle | 10.007s | 7.13s | 1419 | 0 / 441316 |
| Ferrari V12, 1s starter, 10% throttle | 10.007s | 6.58s | 3902 | 10305 / 441300 |
| Default Subaru, 1s starter, 10% throttle | 3.008s | 1.84s | 851 | 0 / 132664 |

These are individual runs on this PC, not performance guarantees or certified
engine behavior. The V12's approximately 2.3% clipping merits listening and gain
inspection before changing its sound settings.
