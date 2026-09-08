# Engine Sim for Windows

Extract the entire ZIP to a writable folder, then open `engine-sim-app.exe`.
Keep the DLLs, `assets`, `es`, and `engine` folders beside it. The application
needs no installer, but the graphics runtime requirements below still apply.

Choose an engine by editing the engine import in `assets/main.mr`. Press Enter
in the simulator to reload it. Failed scripts leave the current engine running;
details are written to `error_log.log` in the working directory.

The initial view fits the engine automatically. After panning or zooming,
press Home to fit it again without restarting the simulation.

The build label and `build-info.txt` identify the Git revision used for the app.
Include that revision in bug reports. A `dirty` suffix marks a build made with
local changes; source archives without Git metadata show `unknown`.

The package includes the Microsoft release runtime. A Windows
x64 PC with DirectX 11 graphics and a working audio output is required.

The GUI also imports `d3dx11_43.dll`, which is not included in this ZIP.
For a missing D3DX library, use Microsoft's
[DirectX End-User Runtimes (June 2010)](https://www.microsoft.com/en-us/download/details.aspx?id=8109).
Having DirectX 11 already installed does not supply that legacy helper library.
The default build excludes unused graphics factories, so Vulkan and DirectX 10
runtimes are no longer required.

A missing DLL can stop Windows from starting the application before it can write
`error_log.log`. `engine-sim-headless.exe` does not import SDL, D3DX or Vulkan;
it uses the bundled Microsoft runtime and does not require a graphics or audio device.

Developer builds generate `delta.conf` beside the executable, pointing at the
checkout. Portable packages use their own adjacent assets without that file.

Developers building Delta's other graphics backends can configure with
`-DENGINE_SIM_D3D11_ONLY=OFF`; this restores the original device factory and its
additional runtime dependencies. Engine Sim itself still selects DirectX 11.
SDL runtime DLLs and their licenses are packaged only with that option disabled.

Startup reuses `assets/assets.ysce` when its layout passes a bounds check and it
is at least as recent as the source geometry, `assets/assets.dia`. Missing,
truncated, unsupported or older compiled files are rebuilt automatically. The
check covers object headers, references, payload lengths and fixed buffer limits;
it is not a full semantic validation of arbitrary geometry. If you change the geometry exporter or restore files
with misleading timestamps, remove `assets.ysce` to force a rebuild.

For simulation and audio statistics without a window, use `engine-sim-headless.exe`.
See `headless.md` for commands and controls.

Video recording is optional and disabled in the default build. In builds made
with `DTV=ON`, Insert starts or stops recording. The app creates `video_capture`
in its working directory and writes `engine_sim_video_capture.mp4` there;
starting another recording overwrites that file. Folder creation errors are
reported in the status panel and `error_log.log`. Odd window dimensions are
scaled down by at most one pixel per axis for H.264 output. If the hardware
encoder cannot initialize, recording retries with software encoding.
GUI capture through that fallback passed in the isolated desktop diagnostic;
hardware recording with the current FFmpeg SDK still needs a compatible driver.

## Building the ZIP

After a Release build, run:

```powershell
cpack --config build/windows/CPackConfig.cmake -C Release -B build/packages
```

The ZIP is written to `build/packages/engine-sim-windows-x64.zip`. For an unpacked
copy, use `cmake --install build/windows --config Release --component Runtime --prefix build/portable`.
The Windows Release CI job also uploads this ZIP as a downloadable artifact.
