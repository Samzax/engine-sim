#include "../include/engine_sim_application.h"

#include <iostream>
#include <cstring>
#include <fstream>
#include <exception>

int WINAPI WinMain(
    _In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPSTR lpCmdLine,
    _In_ int nCmdShow)
{
    (void)nCmdShow;
    (void)hPrevInstance;

    EngineSimApplication application;
    const bool expectEmptyEngine = std::strcmp(lpCmdLine, "--isolated-gui-check-empty") == 0;
    const bool diagnostic = expectEmptyEngine || std::strcmp(lpCmdLine, "--isolated-gui-check") == 0;
    if (diagnostic) {
        char desktopName[256] = {};
        DWORD needed = 0;
        if (!GetUserObjectInformationA(GetThreadDesktop(GetCurrentThreadId()), UOI_NAME,
                desktopName, sizeof(desktopName), &needed)
            || std::strncmp(desktopName, "EngineSimCheck_", 15) != 0) return 2;
        application.setDiagnosticMode();
        std::ofstream report("gui-check.txt");
        report << "GUI diagnostic started; completion pending.\n";
        report.close();
        if (!report) return 3;
    }
    try {
        application.initialize((void *)&hInstance, ysContextObject::DeviceAPI::DirectX11);
        const bool hasEngine = application.getSimulator()->getEngine() != nullptr;
        if (diagnostic && hasEngine == expectEmptyEngine) {
            application.destroy();
            std::ofstream report("gui-check.txt");
            report << "GUI diagnostic failed: expected " << (expectEmptyEngine ? "no engine" : "an engine")
                << " at startup.\n";
            return 4;
        }
        application.run(diagnostic ? 120 : 0);
        const double diagnosticRpm = diagnostic && hasEngine
            ? application.getSimulator()->getEngine()->getRpm() : 0.0;
        application.destroy();
        if (diagnostic) {
            std::ofstream report("gui-check.txt");
            report << "GUI initialization, minimize/restore, ";
            if (hasEngine) report << "successful reload, failed reload preserving the engine, ";
            else report << "empty-engine dashboard, ";
            report << "120 frame-loop iterations and shutdown completed.\n";
            if (hasEngine) report << "Final engine speed with starter engaged: " << diagnosticRpm << " rpm\n";
#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
            report << "GUI video recording and encoder shutdown completed.\n";
#endif
            if (!report) return 3;
        }

        return 0;
    } catch (const std::exception &error) {
        std::ofstream report(diagnostic ? "gui-check.txt" : "runtime-error.txt");
        report << "Engine simulation failed: " << error.what() << '\n';
        report.close();
        if (!diagnostic) MessageBoxA(nullptr, error.what(), "Engine Sim simulation error", MB_OK | MB_ICONERROR);
        return 5;
    }
}
