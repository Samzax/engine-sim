# Off-screen GUI diagnostic

Build the manual launcher and the app on Windows:

```powershell
cmake --build build/windows-x64 --config Release --target engine-sim-app engine-sim-gui-check
cmake --install build/windows-x64 --config Release --component Runtime --prefix build/gui-check-package
```

From a writable directory for diagnostic logs, run the launcher with an absolute
path to the packaged executable:

```powershell
& C:/code/EngineSim/source-review/build/windows-x64/Release/engine-sim-gui-check.exe C:/code/EngineSim/source-review/build/gui-check-package/engine-sim-app.exe
```

The launcher creates a separate Windows desktop and starts the GUI there. It
never calls `SwitchDesktop`, so the diagnostic window and any error dialogs stay
off the input desktop. It waits up to 45 seconds, terminating only its diagnostic
child on timeout. The app accepts the internal diagnostic flag only on a desktop
whose name starts with `EngineSimCheck_`. It mutes audio before playback, skips
Discord initialization, runs 120 frame-loop iterations, and performs normal
shutdown. A successful run writes `gui-check.txt` and exports its final render
target as `gui-check.bmp` in the working directory. The image comes from the
application's GPU texture, not from a desktop screenshot.
The diagnostic enables ignition, holds the starter and sets the speed-control input to 10%
through the simulation API. The report includes final RPM; this exercises the
live gauges and traces but does not claim a stable idle with the starter released.
At frame 20 it minimizes its own window, then restores it at frame 25 and checks
that the client area is positive again. This happens entirely on the isolated
desktop. The Release run passed this cycle on 2026-09-08, and the final exported
frame was inspected after restoration.
At frame 40 it reloads the configured engine through the normal reload and audio
stop/restart path. At frame 80 it deliberately attempts to compile the assets
directory as a script and verifies that the current simulator survives. This
does not modify engine scripts. A successful diagnostic consequently leaves an
expected "Can't find file" entry in `error_log.log`; its process exit code and
completion report determine success.

To check startup without a usable engine, prepare a separate installed copy with
an empty or invalid `assets/main.mr`, then append `--expect-empty-engine` to the
launcher command. This mode requires that no engine loaded, exercises the empty
dashboard and minimize/restore cycle, and skips the engine reload checks. It
does not report starter RPM. The ordinary mode still requires a loaded engine;
either mode exits with code 4 when its startup expectation is wrong. Both modes
remain restricted to the isolated desktop. On 2026-09-08, the normal engine run,
an import-only script, and a syntax-error script passed their respective modes;
the normal mode correctly rejected the import-only script.

When built with `DTV=ON`, the diagnostic also records frames 90 through 109 and
stops the encoder at frame 110. On 2026-09-08, FFmpeg 9.0.1 could not initialize
NVENC with the installed driver, so the app retried in software. The resulting
1904x1040 H.264 file contained 18 frames and decoded without errors; an extracted
frame showed the simulator UI correctly. The SDK DLLs were supplied through the
diagnostic process's PATH. A subsequent video-enabled ZIP bundled those DLLs;
its extracted application passed the same recording check with PATH restricted
to the Windows directories. Hardware encoding with that SDK remains unverified.
Use the launcher's exit code to judge the current run; an older report may remain
after a loader failure. Once the app starts, it replaces that report with a
pending marker before initialization. Startup failures are recorded in
`error_log.log` and diagnostic mode exits without displaying a modal dialog.

Debug and packaged Release passed on the development PC on 2026-09-08. This exercises real GUI,
graphics and audio initialization and the rendering loop. The Release frame
export was inspected for the engine drawing, gauges and labels; it also exposed
and verified a fix for the long reload-error message overflowing its panel.
An additional run with the bundled odd-fire V6 reproduced overlapping engine
name and displacement labels; the exported frame verified that the labels now
fit together with spacing.
This does not verify user interaction or audible output. An off-screen swap chain can
be occluded. Failed Direct3D Present calls now propagate to the application's
frame check and log their HRESULT (plus the removal reason for a reset or removed
device). Non-failure statuses such as occlusion remain accepted. Actual device
removal has not been induced on the development PC.

The original Debug startup failure was traced to HRESULT `0x887A002D`
(`DXGI_ERROR_SDK_COMPONENT_MISSING`): this PC lacks the optional Direct3D debug
layer. A direct device-creation probe succeeded without that flag. The backend
now retries with the standard runtime only for this specific failure and emits
an `OutputDebugString` notice. Other device failures are preserved, and machines
with the debug layer continue to use it. Release passed using both packaged assets and the development
`delta.conf`. That check also found and fixed absolute configuration paths being
incorrectly appended to the executable directory.

The missing-assets failure path was also checked with a separate incomplete
package: it returned exit code 1 immediately, logged the missing paths, and left
the report marked pending rather than reporting success.

The first run found a missing `d3dx10d_43.dll` dependency: upstream linked retail
and debug D3DX libraries together, even in Release. The build now removes the
SDK-only debug libraries for both configurations. The legacy D3DX11 runtime
requirement still applies. The default build now excludes unused
graphics factories and no longer imports D3DX10 or Vulkan.
