# Engine Sim for Windows

Extract the entire ZIP to a writable folder, then open `engine-sim-app.exe`.
Keep the DLLs, `assets`, `es`, and `engine` folders beside it. The application
needs no installer, but the graphics runtime requirements below still apply.

Choose an engine by editing the engine import in `assets/main.mr`. Press Enter
in the simulator to reload it. Failed reloads leave the current engine running.
If the initial script fails, the status instead says no engine is loaded.
Details are written to `error_log.log` in the working directory.

Press Up Arrow or Down Arrow to shift gears. Opposing shift presses received
in the same frame cancel each other, without queuing a shift for the next frame.

Hold 1 through 5 for progressively slower simulation, down to 1/1000 speed.
Release the key to return to normal speed. Fractional physics steps are retained
between display frames, so the slowest settings no longer advance one step per
frame regardless of the requested rate. Audio buffering can still make a small
timing adjustment.

The initial view fits the engine automatically. After panning or zooming,
press Home to fit it again without restarting the simulation.
Hold F1 or F2 to rotate the view; press F3 to reset its angle.

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
in its working directory and writes `engine_sim_video_capture.mp4` there.
If that name exists, it selects the first available numbered name, such as
`engine_sim_video_capture_1.mp4`, preserving earlier recordings. The status panel
shows the chosen filename when recording starts and when the video is saved.
Successful saves also write the full output path to `error_log.log`.
Folder creation errors are
reported in the status panel and `error_log.log`. Odd window dimensions are
scaled down by at most one pixel per axis for H.264 output. If the hardware
encoder cannot initialize, recording retries with software encoding.
GUI capture through that fallback passed in the isolated desktop diagnostic;
hardware recording with the current FFmpeg SDK still needs a compatible driver.

For the tested FFmpeg version, run `./tools/setup-video.ps1`, then configure with
`-DDTV=ON -C build/deps/video-deps.cmake` in addition to the normal Windows
dependency preset. The script downloads a pinned shared SDK, verifies its SHA256,
and extracts it under `build/deps`. CI uses this setup for its video-enabled
Release build and uploads that ZIP separately from the default package.

To supply your own SDK instead, use an FFmpeg shared development distribution and
configure a separate build directory with `-DDTV=ON` and
`-DCMAKE_PREFIX_PATH=C:/path/to/ffmpeg-shared`. Its `lib` directory must contain
the import libraries and its `bin` directory the matching runtime DLLs.
`ENGINE_SIM_FFMPEG_RUNTIME_DIR` and `ENGINE_SIM_FFMPEG_LICENSE` can override the
detected runtime directory and license file. CMake copies the runtime DLLs
beside the development executable and includes them in the
`engine-sim-windows-x64-video.zip` package, along with the supplied license and
build description. The default non-video ZIP retains its existing filename.

## Building the ZIP

After a Release build, run:

```powershell
cpack --config build/windows/CPackConfig.cmake -C Release -B build/packages
```

The ZIP is written to `build/packages/engine-sim-windows-x64.zip`. For an unpacked
copy, use `cmake --install build/windows --config Release --component Runtime --prefix build/portable`.
The Windows Release CI job also uploads this ZIP as a downloadable artifact.
