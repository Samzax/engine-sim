# Convolution performance

The direct convolution filter uses SSE on x64/SSE2 targets to accumulate four
products at a time, with scalar tails and a scalar fallback on other targets.
It retains the existing per-sample interface and adds no block latency or heap
allocation while processing audio.

Local Windows x64 Release measurement (2026-09-08), processing 44,100 samples
through one 10,000-tap channel using `assets/sound-library/archive/engine_01.wav`:

| Implementation | Three wall-clock measurements |
| --- | --- |
| Previous scalar accumulation | 0.301, 0.305, 0.303 seconds |
| SIMD accumulation | 0.073, 0.073, 0.077 seconds |

This is approximately a 4x improvement for this kernel on this PC, not a
whole-application speedup claim. Multi-channel audio, physics, graphics, and
device scheduling still consume time. Floating-point summation order changes;
results are not bit-identical. The regression check compares causal convolution
against double-precision accumulation through ring wrap, including 10,000 taps,
with an absolute tolerance of 0.0001 on bounded random inputs.

To reproduce, configure portable tests as described in `correctness-fixes.md`, then:

```powershell
cmake --build build/audio --config Release --target engine-sim-convolution-benchmark
./build/audio/Release/engine-sim-convolution-benchmark.exe assets/sound-library/archive/engine_01.wav
```

The benchmark runs manually and has no timing pass/fail threshold in CI.
