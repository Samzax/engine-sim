# Correctness fixes and regression checks

This patch is based on `85f7c3b959a908ed5232ede4f1a4ac7eafe6b630`.

Changes:

- Compile/execute failures keep the current engine. Simulator construction and impulse preparation happen before replacing it. The UI has an empty simulator on a failed initial load; diagnostics go to `error_log.log`.
- Removed the unused eight-cylinder scratch array, repaired Debug cleanup, released simulator and script-generated resources, and made simulator/synthesizer cleanup repeatable.
- Audio input/output queues and block state share one mutex. DSP settings are copied once per block; convolution runs outside the queue lock, then completed samples are published. Shutdown wakes blocked workers. The audio consumer must keep draining output while a producer waits for processing.
- Ring buffers distinguish full from empty and drop the oldest sample on overflow. Unavailable output is zero-filled. The application reuses audio scratch storage.
- Missing, silent or unsupported impulse responses use dry audio. The bounded WAV reader accepts mono PCM16 at 44100 Hz and rejects malformed/truncated files before reading sample data.
- Exhaust delay uses the configured physics frequency and resamples queued history when that frequency changes. Zero-step frames report zero intake flow.
- Replaced obsolete synthesizer fixtures with focused regression checks; added Windows Debug/Release and portable sanitizer CI jobs.
- Corrected existing gas-equilibrium tests to transfer the signed flow using the donor's energy (and use the newly calculated outflow). Gaussian tests now compare smoothing against an analytic kernel instead of assuming exact interpolation; physics behavior was not changed to satisfy these tests.

## Portable audio tests

No graphics dependencies or Git submodules are required:

```sh
cmake -S . -B build/audio -DAUDIO_TESTS_ONLY=ON
cmake --build build/audio --config Debug
ctest --test-dir build/audio -C Debug --output-on-failure
```

On Linux, configure a separate build with `-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -g"` or `-DCMAKE_CXX_FLAGS="-fsanitize=thread -g"`. Run AddressSanitizer and ThreadSanitizer separately.

The checks exercise ring wraparound/overflow, convolution tails and empty responses, WAV validation, sample-rate changes with queued delay history, matching synchronous/threaded output, concurrent producer/consumer/settings access, and 100 shutdown/reinitialize cycles. They do not test the graphics window or establish whole-application performance.

## Windows application and integration tests

Use Visual Studio 2022 with the Desktop C++ workload and CMake on PATH:

```powershell
git submodule update --init --recursive
./tools/setup-windows.ps1
cmake -S . -B build/windows -A x64 -C build/deps/windows-deps.cmake
cmake --build build/windows --config Debug --target engine-sim-app engine-sim-test engine-sim-audio-test --parallel 2
ctest --test-dir build/windows -C Debug --output-on-failure -R "^(audio-regressions|GasSystemTests|FunctionTests|SynthesizerTests|SimulatorRegression)"
```

Repeat the build/test commands with `Release`. The setup script downloads fixed versions into `build/deps`; it does not install system-wide packages. The integration tests compile the repository's Hayabusa and Ferrari V12 scripts, exercise simulation/cleanup, and check that script execution cannot return an earlier engine pointer.

Discord Rich Presence remains available in Release. Debug builds omit it because the bundled RPC library was built against the Release C++ runtime.

## Local validation (2026-09-08)

- Windows x64 Debug and Release: application and both project test executables built in each configuration; 29/29 project tests passed in each.
- WSL GCC: the final portable audio suite passed separately under AddressSanitizer + UndefinedBehaviorSanitizer and ThreadSanitizer, with no sanitizer diagnostics.
- The Hayabusa/V12 integration checks exercise script compilation, brief simulation, frequency changes, and repeated cleanup. No graphical/audio-device smoke test or remote GitHub Actions run has been performed.

## Portable launch and packaging follow-up

- Application assets default to folders beside the executable; development builds generate an absolute `delta.conf`. Script compilation now uses the configured asset directory and an explicit script-library path.
- The existing integration checks now execute from the build directory, using absolute fixture/library paths. Debug and Release both passed all 29 checks after this change.
- Registered generated impulse responses with their engine context so repeated uses share the response and engine cleanup releases it.
- CPack builds a Release ZIP containing runtime DLLs, assets, scripts, fonts, shaders, and license files. The local archive was generated and its entries checked; the graphical launch check was interrupted by the user and remains unverified.
- CI runs on fix branches and uploads the Release ZIP. See `portable-release.md` for packaging commands.

## Curve reuse follow-up

Reinitializing a populated `Function` with a smaller capacity previously copied
the old samples into the smaller allocation. Initialization now clears the old
curve first; direct resizing rejects capacities below the existing sample count.
Paired allocations use temporary owners so an allocation failure cannot leak
the first array. Range values reset on reuse and no longer incorrectly include
zero for wholly positive or negative samples. The targeted reuse regression and
the existing project checks passed in Windows Debug and Release (30/30 each).
All six curve tests also passed under WSL AddressSanitizer and
UndefinedBehaviorSanitizer. Unused physics/crankshaft includes were removed from
the Gaussian filter header, allowing this focused check without graphics or
physics dependencies.

## Startup diagnostics follow-up

Startup now checks the configuration, required runtime folders, graphics/shader
and asset initialization results, and returned audio buffer/source pointers.
Failures display a dialog and append details to `error_log.log`, then terminate
with a failure status. This intentionally avoids calling the bundled graphics
engine's cleanup on partially initialized state; Windows reclaims resources on
process exit. Both Windows application configurations build. Dialog behavior and
hardware failure paths remain unverified because desktop control is paused.

Audio-device uploads now check lock/unlock results, skip empty segments, and only
advance the write position after a successful upload. A failed lock previously
left pointer/length outputs unchecked before copying into them. Both application
configurations build; real device-loss recovery still needs desktop verification.
