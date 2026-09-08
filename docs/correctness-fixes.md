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

Journal attachment actions reject a second attachment before overwriting the
journal's owner. Engine selection also rejects duplicate crankshaft instances.
Both mistakes now report script errors instead of silently changing the assembly.
Shared intake and exhaust systems initialize once per engine; cylinders reference
those initialized systems rather than repeating their setup for every cylinder.

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

F1/F2 camera rotation no longer imposes a minimum 5 ms per frame, which made
rotation faster above 200 FPS. It still caps long frames to avoid a jump after a
stall. This camera change does not alter the physics timestep.

Gear shifting consumes both Up and Down transitions each frame. Previously,
an Up press skipped checking Down, leaving a simultaneous downshift pending
until the next frame. Opposing requests now cancel. A standalone check with
the actual keyboard class reproduced requests of `+1, -1` on successive polls
before the change and `0, 0` afterward, without sending OS input. The Release
application build passed; this check does not exercise physical key presses.

### Status-light storage and animation

The dyno panel now clears its four-float status array using the array's own
size. The previous double-based byte count wrote 32 bytes into 16 bytes of
storage, overwriting adjacent members during construction. The firing-order
display also releases its cylinder-light allocation on destruction, including
when a reload replaces the UI.

Firing-order light rise and fade now use elapsed time while preserving the
previous 60 FPS response. For an initially full light with no subsequent firing,
the old fade retained about 46%, 6%, and effectively 0% brightness after 100 ms
at 30, 60, and 240 FPS. The new calculation retains about 6% at all three rates.
Actual firing events remain sampled by the display, so this comparison verifies
the smoothing calculation rather than identical rendered brightness for every
engine event sequence. The Release build and isolated GUI initialization,
reload, minimize/restore, frame-loop, and shutdown check passed.

### Nested connecting rods

Crankshaft references are now resolved after all master-rod links exist, and
initial placement waits until each rod's parent is placed. The previous two
placement passes could leave a deeply nested child attached to its parent's
old position. A three-rod chain listed child-first reproduced a 68 mm initial
joint mismatch and a missing crankshaft reference. After the change, all joints
in that fixture aligned within 1e-9 m and every rod had a crankshaft reference.
The temporary measurement ran before physics and was removed afterward.
All 13 simulator regressions and short load/simulation runs for 20 bundled
engines passed. These checks establish assembly placement, not the physical
realism of arbitrary nested-rod designs.

Initial placement now rejects nonpositive/nonfinite rod lengths and geometry
that cannot reach the cylinder axis above the bank origin, identifying the
cylinder in the error. Previously, failed placement silently left the body at
its default position. A nested-rod fixture with 1 mm rods and a 45-degree bank
was accepted by the old build and reached approximately 153,000 RPM in 10 ms;
the fixed build rejects it before physics starts. All 13 simulator regressions
and 20 bundled short engine runs still passed. This validates the initial
position, not clearance or reachability throughout a complete crank cycle.

Connecting-rod pin coordinates now both subtract the center-of-mass offset
from their midpoint-relative positions. Previously, the big-end sign was
opposite, so a 150 mm rod with a +20 mm offset had only 110 mm between its pins
(190 mm with a -20 mm offset). Positive offsets are toward the little end.
The regression checks pin spacing and the center-of-mass origin for positive,
negative, and zero offsets; it failed before the fix and now passes alongside
the other 13 simulator regressions. Rod destruction also frees its owned
journal-angle array, which previously leaked when engines were destroyed.

### Camshaft lobe counts

Script assembly validation now requires every intake and exhaust camshaft to
have at least as many lobes as its cylinder bank, including both sets of VTEC
cams. This runs before engine allocation. A four-cylinder fixture with only
three intake lobes was previously accepted even though valve evaluation indexed
the missing fourth lobe; it now reports a script error with source locations.
All 14 simulator regressions and short runs of 20 bundled engines passed.

### VTEC vehicle-speed threshold

The previously unused `min_speed` setting now participates in VTEC activation.
The simulator supplies current vehicle speed after each physics step; a newly
loaded engine starts with zero vehicle speed. The regression reproduces the
old activation below a 10 mph threshold and checks activation above it,
deactivation below it, and stationary operation with `min_speed: 0`.
The default script threshold remains 10 mph, so stationary engine-dyno users
who want VTEC activation must explicitly set it to zero. RPM, manifold pressure,
and throttle conditions still apply. All 15 simulator regressions passed.

### Piston wrist-pin offsets

Initial placement now aligns the actual wrist pin with the rod's little end,
chamber volume measures piston height from that pin, and the rendered crown
uses the same compression-height reference. Previously, a nonzero
`wrist_pin_position` affected the constraints but not these calculations
consistently. A Honda fixture with a 10 mm offset reproduced a 10 mm initial
joint gap and a 51.5 cc discrepancy per chamber relative to its actual pin
position. After the fix, the gap was below 2e-14 m and the volume discrepancy
was zero. The temporary measurement was removed afterward. All 15 simulator
regressions passed, and the offset fixture completed the isolated GUI reload,
rendering, and shutdown check; its captured frame was inspected.

### Intake and exhaust gas volumes

Intake initialization rejects nonpositive or nonfinite plenum volume, cross
section area, and derived length. Exhaust initialization applies the same
requirements to collector length, cross section area, and derived volume.
These checks run before initializing the gas systems. Previously, a Honda
fixture with zero plenum volume loaded successfully and reported roughly
9.5e304 RPM after 0.01 seconds. It now fails with a specific geometry error;
a zero-volume exhaust fixture is also rejected. All 15 simulator regressions
and short startup runs of all 20 bundled engine fixtures passed. These checks
cover plenum and collector geometry, not every parameter in an engine script.

Engine-building `invalid_argument` exceptions are translated into script runtime
errors at `set_engine`, after freeing the partially built engine. This is
required for GUI reloads: the headless executable's outer exception handler
alone does not protect the GUI. An isolated-desktop check reloaded the zero-volume
intake fixture into a running engine, reported its source location, preserved
the current simulator, and completed shutdown. The temporary diagnostic path
was removed afterward; all 15 simulator regressions passed.

### Harmonic cam sample count

Harmonic cam lobes now require at least six `steps`, so their `steps - 5`
spacing denominator is positive. Previously `steps: 1` silently generated only
the peak sample, which the curve sampler extended across the entire cycle;
zero steps produced an empty curve. A Honda fixture reproduced successful
loading with one step and now reports a script error at the offending lobe.
The six-step boundary still loads, and all 15 simulator regressions passed,
including the existing cam-duration check. The default remains 100 steps.

### Fractional gauge ranges

Gauge limits and tick intervals now support fractional values. The manifold
gauge's 1.1-bar limit was previously truncated to 1 because the limit was stored
as an integer. Bar mode now uses 0.05-bar minor ticks and 0.1-bar major ticks,
and its near-atmospheric color band spans 0.98 to 1.05 bar instead of wrapping
around the dial from -1 to +1 bar. Tick positions are calculated from an integer
index to avoid accumulated fractional increments. Empty ranges skip needle
normalization and rendering. The Release application built successfully and
the bar setting passed the isolated GUI reload/shutdown check; the captured
frame was inspected for the fractional ticks and corrected color band.

### Application-setting unit defaults

The script defaults for `set_application_settings` now match the canonical
unit names used by the application: `hp`, `lb-ft`, `mph`, `inHg`, and `psi`.
Previously, specifying only a color or pressure setting supplied the uppercase
default `MPH`, which the case-sensitive speed selector interpreted as KPH.
An isolated GUI run with only `pressure_units: "bar"` reproduced KPH before
the change and displayed MPH afterward. Explicitly supplied units are unchanged.

### Positive manifold gauge pressure

PSI and inHg readouts now retain manifold pressure above ambient instead of
clamping it to zero. The existing half-unit near-zero deadband applies to the
magnitude of the reading, so it no longer suppresses all positive values.
An isolated GUI run with a temporary 1.1-atmosphere display input showed the
expected 1.5 PSI reading; previously the conversion and display both forced
that input to zero. The temporary input was removed and the normal application
rebuilt. This is a readout correction, not a change to the intake simulation.

### Fuel-cost estimate

The fuel panel labels its dollar total as an estimate and displays the fixed
assumption of $4.761 USD per US gallon beneath it. The calculation is unchanged;
the rate is not a live fuel price. The Release build and isolated GUI check
passed, and the captured frame confirmed the labels fit the fuel panel.

### Dyno graph smoothing

Torque and power graph smoothing uses elapsed time instead of a fixed 5 percent
blend per rendered frame. Its response at 60 FPS is preserved. A constant-input
calculation over half a second reached 53.67 percent at 30 FPS, 78.54 percent at
60 FPS, and 99.79 percent at 240 FPS before the change; the new filter reached
78.54 percent at all three rates. The Release application built successfully.
Graph sampling frequency and the underlying simulated torque are unchanged.

### Dyno graph sampling cadence

The quarter-second sampling timer now carries overdue time into its next
interval instead of discarding it. A ten-second timer calculation produced
38 samples at 30 and 60 FPS before the change and 40 afterward; 144 FPS also
produced 40. Sampling still occurs on display frames, so individual timestamps
remain frame-quantized. Long frames record only the current reading, without
inventing samples for skipped intervals. Disabling the dyno resets the timer
so the next run samples immediately. The Release application built successfully.

Dyno speed limits are now checked before simulator initialization: both must be
finite and nonnegative, and the maximum must be at least the minimum. A Honda
script with a 9000 RPM minimum and 1000 RPM maximum previously loaded; it now
reports the invalid range. Equal 1000 RPM limits still load for a fixed target.
The Release application/headless builds and all 15 simulator regression checks
passed. This check uses the existing invalid-engine handling for GUI reloads.

The dyno/clutch panel now calls the base UI cleanup routine. Its empty override
previously bypassed deletion of four labeled gauges and their four child gauges
on each dashboard rebuild and shutdown. The ownership path was checked through
`UiElement::destroy` and `LabeledGauge::destroy`; the Release build and hidden GUI
successful reload, failed reload preserving the engine, and shutdown passed.

Stored peak torque, power, and their RPM now update only while the dyno is
enabled. Previously the display filters could continue approaching residual
measurements after switching the dyno off, changing the saved result: for
example, filtered torque 50 and residual input 90 produce 55.714 after one 60 Hz
update. The new condition freezes stored peaks during that period; live filters
continue updating. The calculation was checked directly, and the Release build
and hidden GUI reload/shutdown check passed. The GUI check does not itself
exercise a loaded dyno sweep.

A subsequent isolated GUI trace enabled RPM hold at 1000 RPM for 20 frames,
then disabled the dyno for 15 frames. It measured positive torque while enabled;
peak torque, peak power, and their associated RPM remained exactly unchanged
through every disabled frame. The starter remained engaged, so these readings
are diagnostic loads rather than engine performance measurements. The temporary
trace and control overrides were restored, and the normal executable rebuilt.

Application shutdown now releases the geometry generator's CPU vertex and index
arrays alongside its GPU buffers. Initialization allocates 100000 vertices and
200000 indices, but the shutdown path omitted `GeometryGenerator::destroy` and
its destructor did not free them. The Release build and isolated GUI lifecycle
check, including final shutdown, passed after adding the missing cleanup call.

Removed a discarded `rand()` calculation from each channel's rendered audio
sample. At 44100 Hz this avoids 44100 random draws per exhaust channel per
second. The used noise draw and its filtering remain; the exact random sequence
changes. The existing noise-channel reference was updated to remove the matching
discarded draw, retaining its filter-state comparison. All four selected audio
checks passed, as did the Release builds. No overall speedup is claimed without
a workload benchmark.

Delete now clears saved dyno peaks and torque/power plots without restarting the
engine. Display filter history and plot ranges reset too; an enabled dyno resumes
collecting new measurements immediately. A temporary isolated-desktop diagnostic
enabled RPM hold, disabled it, then called the same clearing functions. Its final
frame showed zero torque/power and an engine still running. The temporary control
override was restored and the normal Release executable rebuilt. This exercised
the clearing functions without injecting a physical Delete key.

The leveler target and gain bounds are now applied once per rendered block from
its existing parameter snapshot, rather than reassigned for every sample. A
2-second headless Hayabusa recording (2 seconds starter, 0.2 speed input) produced
byte-identical WAV files before and after, including 88553 samples. The existing
audio regression executable and Release builds passed. These checks establish
output preservation for that run, not a measured overall performance increase.
