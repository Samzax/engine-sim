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
shutdown. A successful run writes `gui-check.txt` in the working directory.
Use the launcher's exit code to judge the current run; an older report may remain
after a failed run. Startup failures are recorded in `error_log.log`.

Release passed on the development PC on 2026-09-08. This exercises real GUI,
graphics and audio initialization and the rendering loop. It does not verify
visible pixels, user interaction or audible output. An off-screen swap chain can
be occluded, and the upstream graphics backend ignores the HRESULT from Present.

Both Debug and Release built successfully. Debug reached graphics/audio startup
but returned error 19 and the launcher terminated its off-screen error dialog at
the timeout. Debug GUI execution is therefore not verified. The backend requests
the Direct3D debug layer in Debug; the precise device-creation failure has not
been diagnosed. Release passed using both packaged assets and the development
`delta.conf`. That check also found and fixed absolute configuration paths being
incorrectly appended to the executable directory.

The first run found a missing `d3dx10d_43.dll` dependency: upstream linked retail
and debug D3DX libraries together, even in Release. The build now removes the
SDK-only debug libraries for both configurations. Retail legacy DirectX and
Vulkan runtime requirements still apply.
