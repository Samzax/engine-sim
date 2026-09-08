# Correctness fixes and regression checks

This patch is based on `85f7c3b959a908ed5232ede4f1a4ac7eafe6b630`.

Changes:

- Compile/execute failures keep the current engine. Simulator construction and impulse preparation happen before replacing it. The UI has an empty simulator on a failed initial load; diagnostics go to `error_log.log`.
- Removed the unused eight-cylinder scratch array, repaired Debug cleanup, released simulator and script-generated resources, and made simulator/synthesizer cleanup repeatable.
- Audio input/output queues and block state share one mutex. DSP settings are copied once per block; convolution runs outside the queue lock, then completed samples are published. Shutdown wakes blocked workers. The audio consumer must keep draining output while a producer waits for processing.
- Ring buffers distinguish full from empty and drop the oldest sample on overflow. Unavailable output is zero-filled. The application reuses audio scratch storage.
- Missing, silent or unsupported impulse responses use dry audio. The bounded WAV reader accepts mono PCM16 at 44100 Hz and rejects malformed/truncated files before reading sample data.
- Exhaust delay uses the configured physics frequency and resamples queued history when that frequency changes. Frames without a physics step retain the last measured intake flow, avoiding artificial CFM and volumetric-efficiency dips during slow motion.
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

## Initial validation (2026-09-08, historical)

- Windows x64 Debug and Release: application and both project test executables built in each configuration; 29/29 project tests passed in each.
- WSL GCC: the final portable audio suite passed separately under AddressSanitizer + UndefinedBehaviorSanitizer and ThreadSanitizer, with no sanitizer diagnostics.
- The Hayabusa/V12 integration checks exercise script compilation, brief simulation, frequency changes, and repeated cleanup. GUI and remote CI verification were added subsequently; see the current verification scope below.

## Current verification scope (2026-09-08)

- The isolated Windows desktop runner exercises the real GUI with muted audio,
  including minimize/restore, a successful reload, a failed reload preserving
  the engine, rendering and shutdown. Debug at `8a34f03` and subsequent Release
  builds passed. It never switches the user's desktop. See `gui-check.md`.
- The clean portable package at `a1c60531` passed the same GUI check after
  extraction. A copied script with a zero gear ratio produced the expected
  validation error in the packaged GUI; the copied script was restored.
- All 31 selected Debug gas, curve, synthesizer and simulator tests passed at
  `8a34f03`. After the road-drag fix, all five Release simulator regression tests
  passed, including matching deceleration for both rotation directions.
- All 20 bundled runnable engines passed loading and 0.05-second headless runs
  with vehicle/transmission validation enabled. These runs establish startup
  compatibility, not sustained combustion or stable idle.
- DirectSound buffer upload/readback passed separately in Debug and Release;
  see `audio-device-check.md`. Audible quality, device disconnection/recovery,
  GPU removal/reset and hardware video capture remain unverified. These limits
  do not prevent normal background GUI checks.
- Remote CI results belong to individual commits. Local checks and a previous
  successful CI run do not establish that the latest pushed revision is green.

## Portable launch and packaging follow-up

- Application assets default to folders beside the executable; development builds generate an absolute `delta.conf`. Script compilation now uses the configured asset directory and an explicit script-library path.
- The existing integration checks now execute from the build directory, using absolute fixture/library paths. Debug and Release both passed all 29 checks after this change.
- Registered generated impulse responses with their engine context so repeated uses share the response and engine cleanup releases it.
- CPack builds a Release ZIP containing runtime DLLs, assets, scripts, fonts, shaders, and license files. Extracted packages have since passed the isolated GUI launch check described above.
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
process exit. Both Windows application configurations build. The isolated GUI
runner suppresses modal dialogs and checks error exits through logs. Visible
dialog behavior and hardware failure paths remain unverified.

Audio-device uploads now check lock/unlock results, skip empty segments, and only
advance the write position after a successful upload. A failed lock previously
left pointer/length outputs unchecked before copying into them. Both application
configurations build; real device-loss recovery remains unverified.

The pinned Delta Studio backend also passed sample counts as byte counts to
DirectSound's segmented `Unlock` call. `cmake/DeltaAudioFix.cmake` converts both
segment lengths using the existing audio-format helper, matching the backend's
whole-buffer unlock. It compiles a patched build-directory copy, leaves the
submodule clean, and fails configuration if the expected upstream source changes
so the patch must be reviewed. Debug and Release compile the generated source;
actual device playback remains unverified under the background-only constraint.
The subsequent hidden-window diagnostic exercised the actual DirectSound buffer
upload/readback path in Debug and Release without playback or changing focus;
both passed. See `audio-device-check.md` for the exact scope and commands.

The displacement calculation now uses each rod's journal index, converts master
rod journal coordinates relative to its big end, and computes rod-body rotation
with `atan2`. Previously it used a cylinder-bank index and an unnormalized
distance in `acos`, producing incorrect articulated-rod geometry. On the bundled
Radial 9, the old result was 16.488413 L; sampling the simulator's actual piston
placement over a revolution gives 15.939338 L. The corrected calculation agrees
with that independent placement path. This corrects the displayed displacement
and statistics derived from it; it does not change the physics integrator.

Optional video capture now submits a frame only after queue allocation and GPU
readback succeed. A full queue previously returned no frame, but the app still
submitted it, increasing the queue length beyond its capacity. Encoder or
readback errors stop recording and report a status message. The video-enabled
application translation unit passes MSVC syntax checking with the actual
dependency headers. Subsequent full GUI capture passed through software encoding;
see `gui-check.md` for the output validation and dependency setup.

The pinned video queue also retained its stopped flag when initialized for a
second recording, causing empty-queue reads to return immediately and the
encoder worker to spin. `cmake/VideoCaptureFix.cmake` resets this flag in a
generated source copy when video capture is enabled. A standalone check using
the dependency's actual queue reproduced the failure on the second recording;
the patched queue waited for and delivered frames on both recordings. This
checks queue restart behavior independently of FFmpeg or the desktop.

The generated encoder copy also uses const codec/output-format descriptors,
matching current FFmpeg APIs. The original source failed MSVC compilation
against FFmpeg 9.0.1; the patched recorder library and demo linked successfully.
Using the development files from the [Gyan shared build](https://www.gyan.dev/ffmpeg/builds/),
a background check encoded two successive software H.264 recordings with the
same encoder instance. FFprobe counted 30 frames at 64x64 in each file, and
FFmpeg decoded both without errors. This standalone check verifies encoding and
queue reuse; the subsequent GUI capture check is documented in `gui-check.md`. The downloaded
SDK remains under the ignored build directory; the default package still has
video capture disabled.

RGB frame uploads now copy one row at a time using FFmpeg's destination stride.
The original packed copy ignored row padding: a 66x64 software recording of a
uniform gray frame decoded with a black bottom row. With the generated-source
fix, every decoded row had the expected brightness. This also applies to RGBA
window capture when its packed row width differs from FFmpeg's alignment.

Encoder shutdown now sends a flush frame and drains delayed packets before
writing the container trailer. A 30-frame MPEG-2 recording previously decoded
as only 28 frames; after the fix, both successive recordings decoded as all 30.
Trailer-write errors are also propagated to the recorder's error state.

Stream-creation errors now stop setup before opening an uninitialized codec
context. Requesting VC-1 output with the available SDK previously caused an
access violation because no encoder was present; the patched recorder reports
`CouldNotFindEncoder` and shuts down normally.

In builds without video capture, the recording shortcut now reports that the
feature is unavailable instead of silently setting the recording flag. Enabled
builds reject recording dimensions when either dimension is nonpositive, and
start/stop calls guard against repeated transitions. The normal Release build
and video-enabled syntax compilation both passed; the shortcut was not exercised
through keyboard input.

### Harmonic cam duration

The harmonic-lobe generator now inverts its lift equation with the factor of
two outside the gamma exponent. Previously, profiles with gamma other than
one did not match their `duration_at_50_thou` setting. The bundled Hayabusa's
intake and exhaust profiles produced about 0.04354 inches at their specified
0.050-inch timing points. A regression using the compiled engine now verifies
both opening and closing points within 0.00005 inches, allowing for sampled
curve interpolation. This correction changes valve timing and may affect
performance and sound for existing profiles with gamma other than one.

### Ignition at the cycle boundary

Spark detection now handles events on both sides of the 720-to-zero-degree
wrap. Previously, a step spanning that boundary skipped events just before
the wrap by shifting every event into the next cycle. Reverse rotation had
the corresponding error. A regression reproduces both missed sparks and
checks that the corrected events do not fire again on the following step.

### Mixture gauges

Intake AFR now uses the tracked oxygen and inert-gas masses, with the model's
nitrogen approximation for inert gas, and the configured fuel molecular mass.
The old oxygen-only conversion assumed a different oxygen fraction from the
intake supply and overstated the ratio. Exhaust oxygen uses the same configured
fuel mass and reports oxygen even when no unburned fuel remains. These changes
correct the displayed mixture values; the combustion model remains an approximation.

### Slow-motion timing

The frame scheduler now carries fractional physics steps into later frames.
Previously, the audio catch-up adjustment forced at least one step per display
frame, making the 1/1000 speed setting depend on display FPS. Over ten seconds
at a 10 kHz physics frequency, the regression reproduced 300, 600 and 2,400
scheduled steps at 30, 60 and 240 FPS. The corrected scheduler produces about
110 steps in each case: 100 for the requested speed plus the existing 10%
catch-up while the audio queue is below its latency target. This checks step
scheduling; it does not measure audible output at extreme slow-motion settings.

### Script assembly errors and reload cleanup

Engine selection validates required components and rod-journal ownership before
allocating the runtime engine. It rejects reused connecting-rod instances,
cycles in master-rod connections, and ignition wires connected to banks outside
the selected engine. Runtime script errors include source locations in
`error_log.log`. Manual malformed-script cases exercised these errors; all 20
bundled engines passed short load-and-simulate checks after the wiring changes.

Repeated engine, vehicle and transmission selections within one script now
release the superseded objects. Simulator teardown also owns and releases the
physics solvers borrowed by the dependency, and releases the generic solver's
intermediate matrices. Debug lifecycle checks and the isolated GUI reload check
passed. The GUI check deliberately exercises a compile failure; it does not
exercise every malformed assembly through the GUI.

### Window focus and multiple input devices

Windows focus loss now clears cached held keys and mouse buttons before
background release messages are discarded. The UI cancels dragging without
generating a click, including when focus leaves and returns between updates.
The simulation continues running while the window is inactive.

Keyboard and mouse aggregators now consume matching transitions from every
registered device before returning. Previously, boolean short-circuiting left
events on later devices pending for the next poll. A standalone check using
the dependency's actual classes and two in-memory devices reproduced a second
press without new input for both keyboards and mice; both now report only the
first press. Windows mouse input also refreshes every registered mouse's cached
desktop cursor position, so the UI's first-device lookup does not become stale
when another mouse moves.

`cmake/DeltaInputFix.cmake` applies these dependency changes to generated build
copies, leaving the pinned submodule unchanged. Release builds and the isolated
GUI lifecycle check passed. These checks do not establish physical Alt-Tab,
dragging, or multi-mouse behavior on real input devices.
