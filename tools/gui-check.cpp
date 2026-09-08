// Runs the real GUI on an isolated desktop. Never switches the input desktop.
#include <windows.h>
#include <iostream>
#include <string>

int wmain(int argc, wchar_t **argv) {
    const bool expectEmptyEngine = argc == 3 && std::wstring(argv[2]) == L"--expect-empty-engine";
    if (argc != 2 && !expectEmptyEngine) {
        std::cerr << "Usage: engine-sim-gui-check <absolute engine-sim-app.exe path> [--expect-empty-engine]\n";
        return 2;
    }
    const std::wstring name = L"EngineSimCheck_" + std::to_wstring(GetCurrentProcessId());
    HDESK desktop = CreateDesktopW(name.c_str(), nullptr, nullptr, 0,
        DESKTOP_CREATEWINDOW | DESKTOP_READOBJECTS | DESKTOP_WRITEOBJECTS, nullptr);
    if (!desktop) {
        std::cerr << "CreateDesktop failed: " << GetLastError() << '\n';
        return 1;
    }
    std::wstring desktopPath = L"winsta0\\" + name;
    std::wstring command = L"\"" + std::wstring(argv[1]) + L"\" --isolated-gui-check";
    if (expectEmptyEngine) command += L"-empty";
    STARTUPINFOW startup = {};
    startup.cb = sizeof(startup);
    startup.lpDesktop = &desktopPath[0];
    PROCESS_INFORMATION child = {};
    if (!CreateProcessW(argv[1], &command[0], nullptr, nullptr, FALSE, 0,
            nullptr, nullptr, &startup, &child)) {
        std::cerr << "CreateProcess failed: " << GetLastError() << '\n';
        CloseDesktop(desktop);
        return 1;
    }
    const DWORD wait = WaitForSingleObject(child.hProcess, 45000);
    DWORD result = 1;
    if (wait == WAIT_OBJECT_0) GetExitCodeProcess(child.hProcess, &result);
    else {
        std::cerr << "GUI diagnostic timed out or wait failed; terminating diagnostic child.\n";
        TerminateProcess(child.hProcess, 1);
        WaitForSingleObject(child.hProcess, 5000);
    }
    CloseHandle(child.hThread);
    CloseHandle(child.hProcess);
    CloseDesktop(desktop);
    std::cout << "Isolated GUI process exit code: " << result << '\n';
    return result == 0 ? 0 : 1;
}
