#include "../include/engine_sim_application.h"

#include <iostream>
#include <cstring>
#include <fstream>

int WINAPI WinMain(
    _In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPSTR lpCmdLine,
    _In_ int nCmdShow)
{
    (void)nCmdShow;
    (void)hPrevInstance;

    EngineSimApplication application;
    const bool diagnostic = std::strcmp(lpCmdLine, "--isolated-gui-check") == 0;
    if (diagnostic) {
        char desktopName[256] = {};
        DWORD needed = 0;
        if (!GetUserObjectInformationA(GetThreadDesktop(GetCurrentThreadId()), UOI_NAME,
                desktopName, sizeof(desktopName), &needed)
            || std::strncmp(desktopName, "EngineSimCheck_", 15) != 0) return 2;
        application.setDiagnosticMode();
    }
    application.initialize((void *)&hInstance, ysContextObject::DeviceAPI::DirectX11);
    application.run(diagnostic ? 120 : 0);
    application.destroy();
    if (diagnostic) {
        std::ofstream report("gui-check.txt");
        report << "GUI initialization, 120 frame-loop iterations and shutdown completed.\n";
        if (!report) return 3;
    }

    return 0;
}
