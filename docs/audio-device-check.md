# Background DirectSound diagnostic

Build and run manually on Windows with an available audio device:

```powershell
cmake --build build/windows --config Release --target engine-sim-audio-device-check
./build/windows/Release/engine-sim-audio-device-check.exe
```

The diagnostic creates a hidden, non-activating HWND for DirectSound's
cooperative-level requirement. It never shows or activates that window, injects
input, or requests playback. It uses the compiled Delta Studio audio-source
backend to upload and read back samples through whole-buffer locks, contiguous
segments, and a wrap across the buffer boundary. It also reports whether the
foreground window changed during the check.

This target is excluded from normal builds and CTest: it depends on real device
availability. Successful readback validates the local buffer path, not audible
output, device-disconnection recovery, or the graphical application.

Local Windows Debug and Release checks on 2026-09-08: upload/readback passed and the
foreground window was unchanged.
