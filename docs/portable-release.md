# Engine Sim for Windows

Extract the entire ZIP to a writable folder, then open `engine-sim-app.exe`.
Keep the DLLs, `assets`, `es`, and `engine` folders beside it. No installer is required.

Choose an engine by editing the engine import in `assets/main.mr`. Press Enter
in the simulator to reload it. Failed scripts leave the current engine running;
details are written to `error_log.log` in the working directory.

The package includes the Microsoft release runtime and SDL libraries. A Windows
x64 PC with DirectX 11 graphics and a working audio output is required.

Developer builds generate `delta.conf` beside the executable, pointing at the
checkout. Portable packages use their own adjacent assets without that file.

## Building the ZIP

After a Release build, run:

```powershell
cpack --config build/windows/CPackConfig.cmake -C Release -B build/packages
```

The ZIP is written to `build/packages/engine-sim-windows-x64.zip`. For an unpacked
copy, use `cmake --install build/windows --config Release --component Runtime --prefix build/portable`.
The Windows Release CI job also uploads this ZIP as a downloadable artifact.
