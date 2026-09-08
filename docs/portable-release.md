# Engine Sim for Windows

Extract the entire ZIP to a writable folder, then open `engine-sim-app.exe`.
Keep the DLLs, `assets`, `es`, and `engine` folders beside it. The application
needs no installer, but the graphics runtime requirements below still apply.

Choose an engine by editing the engine import in `assets/main.mr`. Press Enter
in the simulator to reload it. Failed scripts leave the current engine running;
details are written to `error_log.log` in the working directory.

The package includes the Microsoft release runtime and SDL libraries. A Windows
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

Startup reuses `assets/assets.ysce` when it is nonempty and at least as recent as
the source geometry, `assets/assets.dia`. Missing, empty or older compiled files
are rebuilt automatically. If you change the geometry exporter or restore files
with misleading timestamps, remove `assets.ysce` to force a rebuild.

For simulation and audio statistics without a window, use `engine-sim-headless.exe`.
See `headless.md` for commands and controls.

## Building the ZIP

After a Release build, run:

```powershell
cpack --config build/windows/CPackConfig.cmake -C Release -B build/packages
```

The ZIP is written to `build/packages/engine-sim-windows-x64.zip`. For an unpacked
copy, use `cmake --install build/windows --config Release --component Runtime --prefix build/portable`.
The Windows Release CI job also uploads this ZIP as a downloadable artifact.
