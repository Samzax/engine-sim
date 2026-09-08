#include "../include/engine_sim_application.h"
#include "engine_sim_build.h"

#include "../include/piston_object.h"
#include "../include/connecting_rod_object.h"
#include "../include/constants.h"
#include "../include/units.h"
#include "../include/crankshaft_object.h"
#include "../include/cylinder_bank_object.h"
#include "../include/cylinder_head_object.h"
#include "../include/ui_button.h"
#include "../include/combustion_chamber_object.h"
#include "../include/csv_io.h"
#include "../include/exhaust_system.h"
#include "../include/feedback_comb_filter.h"
#include "../include/utilities.h"
#include "../include/piston_engine_simulator.h"
#include "../include/wave_reader.h"
#include <memory>
#include <fstream>
#include <filesystem>
#include <cstring>

#include "../scripting/include/compiler.h"

#include <chrono>
#include <stdlib.h>
#include <sstream>

#if ATG_ENGINE_SIM_DISCORD_ENABLED
#include "../dependencies/discord/Discord.h"
#endif

std::string EngineSimApplication::s_buildVersion = "0.1.12a+" ENGINE_SIM_BUILD_REVISION;

namespace {
bool validGeometryCache(const std::filesystem::path &path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return false;
    const auto length = file.tellg();
    if (length < static_cast<std::streamoff>(sizeof(dbasic::CompiledHeader))) return false;
    uint64_t remaining = static_cast<uint64_t>(length);
    file.seekg(0);
    const auto read = [&](void *data, size_t size) {
        if (size > remaining || !file.read(static_cast<char *>(data), size)) return false;
        remaining -= size;
        return true;
    };
    const auto skip = [&](uint64_t size) {
        if (size > remaining) return false;
        file.seekg(static_cast<std::streamoff>(size), std::ios::cur);
        remaining -= size;
        return static_cast<bool>(file);
    };
    dbasic::CompiledHeader scene{};
    if (!read(&scene, sizeof(scene)) || scene.ObjectCount <= 0
        || static_cast<uint64_t>(scene.ObjectCount) > remaining / sizeof(ysGeometryExportFile::ObjectOutputHeader))
        return false;
    uint64_t vertexBytes = 0, indexBytes = 0;
    for (int i = 0; i < scene.ObjectCount; ++i) {
        ysGeometryExportFile::ObjectOutputHeader object{};
        if (!read(&object, sizeof(object))
            || !std::memchr(object.ObjectName, 0, sizeof(object.ObjectName))
            || !std::memchr(object.ObjectMaterial, 0, sizeof(object.ObjectMaterial))
            || object.ParentIndex < -1 || object.ParentIndex >= scene.ObjectCount
            || object.ParentInstanceIndex < -1 || object.ParentInstanceIndex >= scene.ObjectCount)
            return false;
        const auto type = static_cast<ysObjectData::ObjectType>(object.ObjectType);
        if (type == ysObjectData::ObjectType::Geometry) {
            if (object.NumVertices <= 0 || object.VertexDataSize <= 0
                || object.VertexDataSize % object.NumVertices != 0
                || object.NumFaces < 0 || object.NumBones < 0) return false;
            const uint64_t stride = object.VertexDataSize / object.NumVertices;
            vertexBytes = ((vertexBytes + stride - 1) / stride) * stride + object.VertexDataSize;
            indexBytes += static_cast<uint64_t>(object.NumFaces) * 3 * sizeof(unsigned short);
            // Match the pinned loader's fixed GPU/staging capacities.
            if (vertexBytes > 4 * 1024 * 1024 || indexBytes > 1024 * 1024) return false;
            if (!skip(static_cast<uint64_t>(object.VertexDataSize)
                    + static_cast<uint64_t>(object.NumFaces) * 3 * sizeof(unsigned short)
                    + static_cast<uint64_t>(object.NumBones) * sizeof(int))) return false;
        }
        else if (type == ysObjectData::ObjectType::Light) {
            if (!skip(sizeof(ysInterchangeObject::Light))) return false;
        }
        else if (type != ysObjectData::ObjectType::Bone && type != ysObjectData::ObjectType::Group
            && type != ysObjectData::ObjectType::Instance && type != ysObjectData::ObjectType::Empty)
            return false;
    }
    return remaining == 0;
}

bool diagnosticErrors = false;
[[noreturn]] void startupFailure(const std::string &message, bool duringStartup = true) {
    {
        std::ofstream log("error_log.log", std::ios::app);
        log << (duringStartup ? "Startup failed: " : "Rendering failed: ") << message << '\n';
    }
    if (!diagnosticErrors)
        MessageBoxA(nullptr, message.c_str(), duringStartup
            ? "Engine Sim - startup failed" : "Engine Sim - rendering failed", MB_OK | MB_ICONERROR);
    // DeltaEngine::Destroy and its destructors require complete initialization.
    // Terminate this failed startup without unwinding partially created graphics
    // objects; Windows reclaims the process resources.
    std::exit(EXIT_FAILURE);
}

void checkStartup(ysError error, const char *stage) {
    if (error != ysError::None) {
        startupFailure(std::string(stage) + " failed (error "
            + std::to_string(static_cast<int>(error)) + ").\n"
            "Check that the complete package was extracted and DirectX 11 is available.");
    }
}

void checkFrame(ysError error, const char *stage) {
    if (error != ysError::None) {
        startupFailure(std::string(stage) + " failed (error "
            + std::to_string(static_cast<int>(error))
            + "). See error_log.log for graphics error details.", false);
    }
}

void saveDiagnosticFrame(dbasic::DeltaEngine &engine) {
    ysRenderTarget *target = engine.GetScreenRenderTarget();
    const int width = target->GetWidth(), height = target->GetHeight();
    if (width <= 0 || height <= 0)
        startupFailure("Diagnostic render target has invalid dimensions.");
    std::vector<uint8_t> pixels(static_cast<size_t>(width) * height * 4);
    checkFrame(engine.GetDevice()->ReadRenderTarget(target, pixels.data()), "Read diagnostic frame");
    // Direct3D returns top-down RGBA; a 32-bit BMP stores top-down BGRA when
    // its height is negative. This exports only the application's render target.
    for (size_t i = 0; i < pixels.size(); i += 4) std::swap(pixels[i], pixels[i + 2]);
    BITMAPFILEHEADER fileHeader{};
    BITMAPINFOHEADER bitmapHeader{};
    fileHeader.bfType = 0x4d42;
    fileHeader.bfOffBits = sizeof(fileHeader) + sizeof(bitmapHeader);
    fileHeader.bfSize = fileHeader.bfOffBits + static_cast<DWORD>(pixels.size());
    bitmapHeader.biSize = sizeof(bitmapHeader);
    bitmapHeader.biWidth = width;
    bitmapHeader.biHeight = -height;
    bitmapHeader.biPlanes = 1;
    bitmapHeader.biBitCount = 32;
    bitmapHeader.biCompression = BI_RGB;
    bitmapHeader.biSizeImage = static_cast<DWORD>(pixels.size());
    std::ofstream image("gui-check.bmp", std::ios::binary | std::ios::trunc);
    image.write(reinterpret_cast<const char *>(&fileHeader), sizeof(fileHeader));
    image.write(reinterpret_cast<const char *>(&bitmapHeader), sizeof(bitmapHeader));
    image.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
    image.close();
    if (!image) startupFailure("Could not write gui-check.bmp.");
}
}

void EngineSimApplication::setDiagnosticMode() {
    m_diagnosticMode = true;
    diagnosticErrors = true;
}

EngineSimApplication::EngineSimApplication() {
    m_assetPath = "";

    m_geometryVertexBuffer = nullptr;
    m_geometryIndexBuffer = nullptr;

    m_paused = false;
    m_recording = false;
    m_screenResolutionIndex = 0;
    for (int i = 0; i < ScreenResolutionHistoryLength; ++i) {
        m_screenResolution[i][0] = m_screenResolution[i][1] = 0;
    }

    m_background = ysColor::srgbiToLinear(0x0E1012);
    m_foreground = ysColor::srgbiToLinear(0xFFFFFF);
    m_shadow = ysColor::srgbiToLinear(0x0E1012);
    m_highlight1 = ysColor::srgbiToLinear(0xEF4545);
    m_highlight2 = ysColor::srgbiToLinear(0xFFFFFF);
    m_pink = ysColor::srgbiToLinear(0xF394BE);
    m_red = ysColor::srgbiToLinear(0xEE4445);
    m_orange = ysColor::srgbiToLinear(0xF4802A);
    m_yellow = ysColor::srgbiToLinear(0xFDBD2E);
    m_blue = ysColor::srgbiToLinear(0x77CEE0);
    m_green = ysColor::srgbiToLinear(0xBDD869);

    m_displayHeight = (float)units::distance(2.0, units::foot);
    m_outputAudioBuffer = nullptr;
    m_audioSource = nullptr;

    m_torque = 0;
    m_dynoSpeed = 0;

    m_simulator = nullptr;
    m_engineView = nullptr;
    m_rightGaugeCluster = nullptr;
    m_temperatureGauge = nullptr;
    m_oscCluster = nullptr;
    m_performanceCluster = nullptr;
    m_loadSimulationCluster = nullptr;
    m_mixerCluster = nullptr;
    m_infoCluster = nullptr;
    m_iceEngine = nullptr;
    m_mainRenderTarget = nullptr;

    m_vehicle = nullptr;
    m_transmission = nullptr;

    m_oscillatorSampleOffset = 0;
    m_gameWindowHeight = 256;
    m_screenWidth = 256;
    m_screenHeight = 256;
    m_screen = 0;
    m_viewParameters.Layer0 = 0;
    m_viewParameters.Layer1 = 0;

    m_displayAngle = 0.0f;
}

EngineSimApplication::~EngineSimApplication() {
    /* void */
}

void EngineSimApplication::initialize(void *instance, ysContextObject::DeviceAPI api) {
    dbasic::Path modulePath = dbasic::GetModulePath();
    dbasic::Path confPath = modulePath.Append("delta.conf");

    std::string enginePath = modulePath.Append("engine").ToString();
    m_assetPath = modulePath.Append("assets").ToString();
    if (confPath.Exists()) {
        std::fstream confFile(confPath.ToString(), std::ios::in);

        if (!std::getline(confFile, enginePath) || !std::getline(confFile, m_assetPath)
            || enginePath.empty() || m_assetPath.empty()) {
            startupFailure("Invalid delta.conf: expected an engine directory and an assets directory on separate lines.");
        }
        if (!std::filesystem::path(enginePath).is_absolute())
            enginePath = modulePath.Append(enginePath).ToString();
        if (!std::filesystem::path(m_assetPath).is_absolute())
            m_assetPath = modulePath.Append(m_assetPath).ToString();

        confFile.close();
    }

    if (!dbasic::Path(enginePath).Append("fonts/dc_font_consolas.png").Exists()
        || !dbasic::Path(enginePath).Append("shaders").Exists()
        || !dbasic::Path(m_assetPath).Exists()) {
        startupFailure("Required fonts, shaders or assets are missing.\nExtract the complete package beside the executable, or correct delta.conf.\nEngine: "
            + enginePath + "\nAssets: " + m_assetPath);
    }
    m_engine.GetConsole()->SetDefaultFontDirectory(enginePath + "/fonts/");

    const std::string shaderPath = enginePath + "/shaders/";
    const std::string winTitle = "Engine Sim | AngeTheGreat | v" + s_buildVersion;
    dbasic::DeltaEngine::GameEngineSettings settings;
    settings.API = api;
    settings.DepthBuffer = false;
    settings.Instance = instance;
    settings.ShaderDirectory = shaderPath.c_str();
    settings.WindowTitle = winTitle.c_str();
    settings.WindowPositionX = 0;
    settings.WindowPositionY = 0;
    settings.WindowStyle = ysWindow::WindowStyle::Windowed;
    settings.WindowWidth = 1920;
    settings.WindowHeight = 1080;

    checkStartup(m_engine.CreateGameWindow(settings), "Graphics/audio initialization");

    checkStartup(m_engine.GetDevice()->CreateSubRenderTarget(
        &m_mainRenderTarget,
        m_engine.GetScreenRenderTarget(),
        0,
        0,
        0,
        0), "Main render target creation");

    checkStartup(m_engine.InitializeShaderSet(&m_shaderSet), "Shader set initialization");
    checkStartup(m_shaders.Initialize(
        &m_shaderSet,
        m_mainRenderTarget,
        m_engine.GetScreenRenderTarget(),
        m_engine.GetDefaultShaderProgram(),
        m_engine.GetDefaultInputLayout()), "Application shader initialization");
    checkStartup(m_engine.InitializeConsoleShaders(&m_shaderSet), "Console shader initialization");
    m_engine.SetShaderSet(&m_shaderSet);

    m_shaders.SetClearColor(ysColor::srgbiToLinear(0x34, 0x98, 0xdb));

    m_assetManager.SetEngine(&m_engine);

    checkStartup(m_engine.GetDevice()->CreateIndexBuffer(
        &m_geometryIndexBuffer, sizeof(unsigned short) * 200000, nullptr), "Index buffer creation");
    checkStartup(m_engine.GetDevice()->CreateVertexBuffer(
        &m_geometryVertexBuffer, sizeof(dbasic::Vertex) * 100000, nullptr), "Vertex buffer creation");

    m_geometryGenerator.initialize(100000, 200000);

    initialize();
}

void EngineSimApplication::initialize() {
    m_shaders.SetClearColor(ysColor::srgbiToLinear(0x34, 0x98, 0xdb));
    const std::filesystem::path geometryPath = std::filesystem::path(m_assetPath) / "assets";
    const std::filesystem::path geometrySource = geometryPath.string() + ".dia";
    const std::filesystem::path geometryCache = geometryPath.string() + ".ysce";
    std::error_code cacheError;
    const auto cacheTime = std::filesystem::last_write_time(geometryCache, cacheError);
    bool rebuildGeometry = static_cast<bool>(cacheError);
    if (!rebuildGeometry) {
        const auto sourceTime = std::filesystem::last_write_time(geometrySource, cacheError);
        rebuildGeometry = static_cast<bool>(cacheError) || sourceTime > cacheTime;
    }
    if (!rebuildGeometry) {
        rebuildGeometry = !validGeometryCache(geometryCache);
    }
    if (rebuildGeometry)
        checkStartup(m_assetManager.CompileInterchangeFile(geometryPath.string().c_str(), 1.0f, true), "Asset compilation");
    if (rebuildGeometry && !validGeometryCache(geometryCache))
        startupFailure("Compiled geometry is incomplete or exceeds the supported buffer sizes.");
    checkStartup(m_assetManager.LoadSceneFile((m_assetPath + "/assets").c_str(), true), "Asset loading");

    m_textRenderer.SetEngine(&m_engine);
    m_textRenderer.SetRenderer(m_engine.GetUiRenderer());
    m_textRenderer.SetFont(m_engine.GetConsole()->GetFont());

    // Keep a valid empty simulator for the UI if the first script fails.
    m_simulator = new PistonEngineSimulator;
    loadScript();
    m_audioScratch.resize(44100);

    m_audioBuffer.initialize(44100, 44100);
    m_audioBuffer.m_writePointer = (int)(44100 * 0.1);

    ysAudioParameters params;
    params.m_bitsPerSample = 16;
    params.m_channelCount = 1;
    params.m_sampleRate = 44100;
    if (m_engine.GetAudioDevice() == nullptr) startupFailure("No audio output device is available.");
    m_outputAudioBuffer =
        m_engine.GetAudioDevice()->CreateBuffer(&params, 44100);
    if (m_outputAudioBuffer == nullptr) startupFailure("Could not create the output audio buffer.");

    m_audioSource = m_engine.GetAudioDevice()->CreateSource(m_outputAudioBuffer);
    if (m_audioSource == nullptr) startupFailure("Could not create the output audio source.");
    checkStartup(m_audioSource->SetVolume(m_diagnosticMode ? 0.0f : 1.0f), "Audio volume initialization");
    m_audioSource->SetMode((m_simulator->getEngine() != nullptr)
        ? ysAudioSource::Mode::Loop
        : ysAudioSource::Mode::Stop);
    m_audioSource->SetPan(0.0f);

#ifdef ATG_ENGINE_SIM_DISCORD_ENABLED
    if (!m_diagnosticMode) {
    // Create a global instance of discord-rpc
    CDiscord::CreateInstance();

    // Enable it, this needs to be set via a config file of some sort. 
    GetDiscordManager()->SetUseDiscord(true);
    DiscordRichPresence passMe = { 0 };

    std::string engineName = (m_iceEngine != nullptr)
        ? m_iceEngine->getName()
        : "Broken Engine";

    GetDiscordManager()->SetStatus(passMe, engineName, s_buildVersion);
    }
#endif /* ATG_ENGINE_SIM_DISCORD_ENABLED */
}

void EngineSimApplication::process(float frame_dt) {
    frame_dt = static_cast<float>(clamp(frame_dt, 1 / 200.0f, 1 / 30.0f));

    double speed = 1.0 / 1.0;
    if (m_engine.IsKeyDown(ysKey::Code::N1)) {
        speed = 1 / 10.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N2)) {
        speed = 1 / 100.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N3)) {
        speed = 1 / 200.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N4)) {
        speed = 1 / 500.0;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N5)) {
        speed = 1 / 1000.0;
    }

    if (m_engine.IsKeyDown(ysKey::Code::F1)) {
        m_displayAngle += frame_dt * 1.0f;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::F2)) {
        m_displayAngle -= frame_dt * 1.0f;
    }
    else if (m_engine.ProcessKeyDown(ysKey::Code::F3)) {
        m_displayAngle = 0.0f;
    }

    m_simulator->setSimulationSpeed(speed);

    const double avgFramerate = clamp(m_engine.GetAverageFramerate(), 30.0f, 1000.0f);
    m_simulator->startFrame(1 / avgFramerate);

    auto proc_t0 = std::chrono::steady_clock::now();
    const int iterationCount = m_simulator->getFrameIterationCount();
    while (m_simulator->simulateStep()) {
        m_oscCluster->sample();
    }

    auto proc_t1 = std::chrono::steady_clock::now();

    m_simulator->endFrame();

    auto duration = proc_t1 - proc_t0;
    if (iterationCount > 0) {
        m_performanceCluster->addTimePerTimestepSample(
            (duration.count() / 1E9) / iterationCount);
    }

    const SampleOffset safeWritePosition = m_audioSource->GetCurrentWritePosition();
    const SampleOffset writePosition = m_audioBuffer.m_writePointer;

    SampleOffset targetWritePosition =
        m_audioBuffer.getBufferIndex(safeWritePosition, (int)(44100 * 0.1));
    SampleOffset maxWrite = m_audioBuffer.offsetDelta(writePosition, targetWritePosition);

    SampleOffset currentLead = m_audioBuffer.offsetDelta(safeWritePosition, writePosition);
    SampleOffset newLead = m_audioBuffer.offsetDelta(safeWritePosition, targetWritePosition);

    if (currentLead > 44100 * 0.5) {
        m_audioBuffer.m_writePointer = m_audioBuffer.getBufferIndex(safeWritePosition, (int)(44100 * 0.05));
        currentLead = m_audioBuffer.offsetDelta(safeWritePosition, m_audioBuffer.m_writePointer);
        maxWrite = m_audioBuffer.offsetDelta(m_audioBuffer.m_writePointer, targetWritePosition);
    }

    if (currentLead > newLead) {
        maxWrite = 0;
    }

    int16_t *samples = m_audioScratch.data();
    const int readSamples = m_simulator->readAudioOutput(maxWrite, samples);

    for (SampleOffset i = 0; i < (SampleOffset)readSamples && i < maxWrite; ++i) {
        const int16_t sample = samples[i];
        if (m_oscillatorSampleOffset % 4 == 0) {
            m_oscCluster->getAudioWaveformOscilloscope()->addDataPoint(
                m_oscillatorSampleOffset,
                sample / (float)(INT16_MAX));
        }

        m_audioBuffer.writeSample(sample, m_audioBuffer.m_writePointer, (int)i);

        m_oscillatorSampleOffset = (m_oscillatorSampleOffset + 1) % (44100 / 10);
    }

    if (readSamples > 0) {
        SampleOffset size0 = 0, size1 = 0;
        void *data0 = nullptr, *data1 = nullptr;
        const ysError lockResult = m_audioSource->LockBufferSegment(
            m_audioBuffer.m_writePointer, readSamples, &data0, &size0, &data1, &size1);
        if (lockResult != ysError::None) {
            m_infoCluster->setLogMessage("Audio buffer unavailable; retrying next frame");
            return;
        }

        if (size0 > 0) m_audioBuffer.copyBuffer(
            reinterpret_cast<int16_t *>(data0), m_audioBuffer.m_writePointer, size0);
        if (size1 > 0) m_audioBuffer.copyBuffer(
            reinterpret_cast<int16_t *>(data1),
            m_audioBuffer.getBufferIndex(m_audioBuffer.m_writePointer, size0),
            size1);

        if (m_audioSource->UnlockBufferSegments(data0, size0, data1, size1) == ysError::None) {
            m_audioBuffer.commitBlock(readSamples);
        }
        else {
            m_infoCluster->setLogMessage("Audio buffer upload failed; retrying next frame");
        }
    }

    m_performanceCluster->addInputBufferUsageSample(
        (double)m_simulator->getSynthesizerInputLatency() / m_simulator->getSynthesizerInputLatencyTarget());
    m_performanceCluster->addAudioLatencySample(
        m_audioBuffer.offsetDelta(m_audioSource->GetCurrentWritePosition(), m_audioBuffer.m_writePointer) / (44100 * 0.1));
}

void EngineSimApplication::render() {
    for (SimulationObject *object : m_objects) {
        object->generateGeometry();
    }

    m_viewParameters.Sublayer = 0;
    for (SimulationObject *object : m_objects) {
        object->render(&getViewParameters());
    }

    m_viewParameters.Sublayer = 1;
    for (SimulationObject *object : m_objects) {
        object->render(&getViewParameters());
    }

    m_viewParameters.Sublayer = 2;
    for (SimulationObject *object : m_objects) {
        object->render(&getViewParameters());
    }

    m_uiManager.render();
}

float EngineSimApplication::pixelsToUnits(float pixels) const {
    const float f = m_displayHeight / m_engineView->m_bounds.height();
    return pixels * f;
}

float EngineSimApplication::unitsToPixels(float units) const {
    const float f = m_engineView->m_bounds.height() / m_displayHeight;
    return units * f;
}

void EngineSimApplication::run(int maxFrames) {
    int frames = 0;
    while (true) {
        checkFrame(m_engine.StartFrame(), "Start frame");

        if (!m_engine.IsOpen()) break;
        if (m_engine.ProcessKeyDown(ysKey::Code::Escape)) {
            break;
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::Return)
            || (m_diagnosticMode && (frames == 40 || frames == 80))) {
            Simulator *previous = m_simulator;
            m_audioSource->SetMode(ysAudioSource::Mode::Stop);
            // A directory cannot be an engine script: exercise a failed load
            // without changing the user's files, through the same audio path.
            const bool expectedFailure = m_diagnosticMode && frames == 80;
            loadScript(expectedFailure ? m_assetPath : "");
            if (m_diagnosticMode && (m_iceEngine == nullptr
                || (expectedFailure ? m_simulator != previous : m_simulator == previous)))
                startupFailure("GUI diagnostic reload did not preserve or replace the engine as expected.");
            if (m_simulator->getEngine() != nullptr) {
                m_audioSource->SetMode(ysAudioSource::Mode::Loop);
            }
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::Tab)) {
            m_screen++;
            if (m_screen > 2) m_screen = 0;
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::F)) {
            if (m_engine.GetGameWindow()->GetWindowStyle() != ysWindow::WindowStyle::Fullscreen) {
                m_engine.GetGameWindow()->SetWindowStyle(ysWindow::WindowStyle::Fullscreen);
                m_infoCluster->setLogMessage("Entered fullscreen mode");
            }
            else {
                m_engine.GetGameWindow()->SetWindowStyle(ysWindow::WindowStyle::Windowed);
                m_infoCluster->setLogMessage("Exited fullscreen mode");
            }
        }

        m_gameWindowHeight = m_engine.GetGameWindow()->GetGameHeight();
        m_screenHeight = m_engine.GetGameWindow()->GetScreenHeight();
        m_screenWidth = m_engine.GetGameWindow()->GetScreenWidth();

        updateScreenSizeStability();

        processEngineInput();
        if (m_diagnosticMode && m_iceEngine != nullptr) {
            m_iceEngine->getIgnitionModule()->m_enabled = true;
            m_iceEngine->setThrottle(0.1);
        }

        if (m_engine.ProcessKeyDown(ysKey::Code::Insert) &&
            m_engine.GetGameWindow()->IsActive()) {
            if (!isRecording() && readyToRecord()) {
                startRecording();
            }
            else if (isRecording()) {
                stopRecording();
            }
        }

        if (isRecording() && !readyToRecord()) {
            stopRecording();
        }

        if (!m_paused || m_engine.ProcessKeyDown(ysKey::Code::Right)) {
            process(m_engine.GetFrameLength());
        }

        m_uiManager.update(m_engine.GetFrameLength());

        renderScene();

        checkFrame(m_engine.EndFrame(), "Render frame");
        if (m_diagnosticMode && frames == maxFrames - 1) saveDiagnosticFrame(m_engine);
        if (maxFrames > 0 && ++frames >= maxFrames) break;

        if (isRecording()) {
            recordFrame();
        }
    }

    if (maxFrames > 0 && frames != maxFrames) startupFailure("GUI diagnostic window closed before completing its frames.");

    if (isRecording()) {
        stopRecording();
    }

    m_simulator->endAudioRenderingThread();
}

void EngineSimApplication::destroy() {
    if (m_simulator != nullptr) {
        m_simulator->releaseSimulation();
        delete m_simulator;
        m_simulator = nullptr;
    }
    m_uiManager.destroy();
    destroyObjects();
    if (m_iceEngine != nullptr) {
        m_iceEngine->destroy();
        delete m_iceEngine;
        m_iceEngine = nullptr;
    }
    delete m_vehicle;
    m_vehicle = nullptr;
    delete m_transmission;
    m_transmission = nullptr;
    m_shaderSet.Destroy();

    m_engine.GetDevice()->DestroyGPUBuffer(m_geometryVertexBuffer);
    m_engine.GetDevice()->DestroyGPUBuffer(m_geometryIndexBuffer);

    m_assetManager.Destroy();
    m_engine.Destroy();

    m_audioBuffer.destroy();
}

bool EngineSimApplication::loadEngine(
    Engine *engine,
    Vehicle *vehicle,
    Transmission *transmission)
{
    if (engine == nullptr) {
        std::ofstream log("error_log.log", std::ios::app);
        log << "No engine was provided.\n";
        return false;
    }

    std::unique_ptr<Simulator> replacement;
    try {
        replacement.reset(engine->createSimulator(vehicle, transmission));
        engine->calculateDisplacement();
        auto audioParams = replacement->synthesizer().getAudioParameters();
        audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
        audioParams.airNoise = static_cast<float>(engine->getInitialNoise());
        audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
        replacement->synthesizer().setAudioParameters(audioParams);

        for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
            ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();
            std::vector<int16_t> impulse;
            const bool loaded = response != nullptr && readImpulseWave(response->getFilename(), 44100, impulse);
            replacement->synthesizer().initializeImpulseResponse(
                loaded ? impulse.data() : nullptr, static_cast<unsigned>(impulse.size()),
                response != nullptr ? response->getVolume() : 1.0f, i);
            if (!loaded) {
                std::ofstream log("error_log.log", std::ios::app);
                log << "Exhaust channel " << i << ": using dry audio; impulse response "
                    << (response != nullptr ? response->getFilename() : "<not configured>")
                    << " is missing or unsupported. Expected mono PCM16 at 44100 Hz.\n";
            }
        }

    } catch (const std::exception &error) {
        std::ofstream log("error_log.log", std::ios::app);
        log << "Unable to load replacement engine: " << error.what() << '\n';
        return false;
    }

    destroyObjects();

    if (m_simulator != nullptr) {
        m_simulator->releaseSimulation();
        delete m_simulator;
    }

    if (m_vehicle != nullptr) {
        delete m_vehicle;
        m_vehicle = nullptr;
    }

    if (m_transmission != nullptr) {
        delete m_transmission;
        m_transmission = nullptr;
    }

    if (m_iceEngine != nullptr) {
        m_iceEngine->destroy();
        delete m_iceEngine;
    }

    m_iceEngine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;

    m_simulator = replacement.release();

    createObjects(engine);

    m_viewParameters.Layer1 = engine->getMaxDepth();
    m_viewParameters.Layer0 = (std::max)(0,
        (std::min)(m_viewParameters.Layer0, m_viewParameters.Layer1 - 1));
    m_simulator->startAudioRenderingThread();
    return true;
}

void EngineSimApplication::drawGenerated(
    const GeometryGenerator::GeometryIndices &indices,
    int layer)
{
    drawGenerated(indices, layer, m_shaders.GetRegularFlags());
}

void EngineSimApplication::drawGeneratedUi(
    const GeometryGenerator::GeometryIndices &indices,
    int layer)
{
    drawGenerated(indices, layer, m_shaders.GetUiFlags());
}

void EngineSimApplication::drawGenerated(
    const GeometryGenerator::GeometryIndices &indices,
    int layer,
    dbasic::StageEnableFlags flags)
{
    m_engine.DrawGeneric(
        flags,
        m_geometryIndexBuffer,
        m_geometryVertexBuffer,
        sizeof(dbasic::Vertex),
        indices.BaseIndex,
        indices.BaseVertex,
        indices.FaceCount,
        false,
        layer);
}

void EngineSimApplication::configure(const ApplicationSettings &settings) {
    m_applicationSettings = settings;

    if (settings.startFullscreen) {
        m_engine.GetGameWindow()->SetWindowStyle(ysWindow::WindowStyle::Fullscreen);
    }

    m_background = ysColor::srgbiToLinear(m_applicationSettings.colorBackground);
    m_foreground = ysColor::srgbiToLinear(m_applicationSettings.colorForeground);
    m_shadow = ysColor::srgbiToLinear(m_applicationSettings.colorShadow);
    m_highlight1 = ysColor::srgbiToLinear(m_applicationSettings.colorHighlight1);
    m_highlight2 = ysColor::srgbiToLinear(m_applicationSettings.colorHighlight2);
    m_pink = ysColor::srgbiToLinear(m_applicationSettings.colorPink);
    m_red = ysColor::srgbiToLinear(m_applicationSettings.colorRed);
    m_orange = ysColor::srgbiToLinear(m_applicationSettings.colorOrange);
    m_yellow = ysColor::srgbiToLinear(m_applicationSettings.colorYellow);
    m_blue = ysColor::srgbiToLinear(m_applicationSettings.colorBlue);
    m_green = ysColor::srgbiToLinear(m_applicationSettings.colorGreen);
}

void EngineSimApplication::createObjects(Engine *engine) {
    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        ConnectingRodObject *rodObject = new ConnectingRodObject;
        rodObject->initialize(this);
        rodObject->m_connectingRod = engine->getConnectingRod(i);
        m_objects.push_back(rodObject);

        PistonObject *pistonObject = new PistonObject;
        pistonObject->initialize(this);
        pistonObject->m_piston = engine->getPiston(i);
        m_objects.push_back(pistonObject);

        CombustionChamberObject *ccObject = new CombustionChamberObject;
        ccObject->initialize(this);
        ccObject->m_chamber = m_iceEngine->getChamber(i);
        m_objects.push_back(ccObject);
    }

    for (int i = 0; i < engine->getCrankshaftCount(); ++i) {
        CrankshaftObject *crankshaftObject = new CrankshaftObject;
        crankshaftObject->initialize(this);
        crankshaftObject->m_crankshaft = engine->getCrankshaft(i);
        m_objects.push_back(crankshaftObject);
    }

    for (int i = 0; i < engine->getCylinderBankCount(); ++i) {
        CylinderBankObject *cbObject = new CylinderBankObject;
        cbObject->initialize(this);
        cbObject->m_bank = engine->getCylinderBank(i);
        cbObject->m_head = engine->getHead(i);
        m_objects.push_back(cbObject);

        CylinderHeadObject *chObject = new CylinderHeadObject;
        chObject->initialize(this);
        chObject->m_head = engine->getHead(i);
        chObject->m_engine = engine;
        m_objects.push_back(chObject);
    }
}

void EngineSimApplication::destroyObjects() {
    for (SimulationObject *object : m_objects) {
        object->destroy();
        delete object;
    }

    m_objects.clear();
}

const SimulationObject::ViewParameters &
    EngineSimApplication::getViewParameters() const
{
    return m_viewParameters;
}

void EngineSimApplication::loadScript(const std::string &scriptPath) {
    Engine *engine = nullptr;
    Vehicle *vehicle = nullptr;
    Transmission *transmission = nullptr;
    ApplicationSettings settings;
    bool executed = false;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    es_script::Compiler compiler;
    compiler.initialize(dbasic::Path(m_assetPath).Append("../es").ToString());
    const bool compiled = compiler.compile(scriptPath.empty() ? m_assetPath + "/main.mr" : scriptPath);
    if (compiled) {
        const es_script::Compiler::Output output = compiler.execute();
        settings = output.applicationSettings;
        executed = output.success;

        engine = output.engine;
        vehicle = output.vehicle;
        transmission = output.transmission;
    }
    else {
        engine = nullptr;
        vehicle = nullptr;
        transmission = nullptr;
    }

    compiler.destroy();
#endif /* ATG_ENGINE_SIM_PIRANHA_ENABLED */

    if (!executed || engine == nullptr) {
        if (engine != nullptr) { engine->destroy(); delete engine; }
        delete vehicle;
        delete transmission;
        if (m_infoCluster == nullptr) refreshUserInterface();
        m_infoCluster->setLogMessage("Script failed; keeping current engine. See error_log.log");
        return;
    }

    if (vehicle == nullptr) {
        Vehicle::Parameters vehParams;
        vehParams.mass = units::mass(1597, units::kg);
        vehParams.diffRatio = 3.42;
        vehParams.tireRadius = units::distance(10, units::inch);
        vehParams.dragCoefficient = 0.25;
        vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vehParams.rollingResistance = 2000.0;
        vehicle = new Vehicle;
        vehicle->initialize(vehParams);
    }

    if (transmission == nullptr) {
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission;
        transmission->initialize(tParams);
    }

    if (!loadEngine(engine, vehicle, transmission)) {
        engine->destroy();
        delete engine;
        delete vehicle;
        delete transmission;
        if (m_infoCluster == nullptr) refreshUserInterface();
        m_infoCluster->setLogMessage("Invalid engine; keeping current engine. See error_log.log");
        return;
    }
    configure(settings);
    refreshUserInterface();
}

void EngineSimApplication::processEngineInput() {
    if (m_iceEngine == nullptr) {
        return;
    }

    const float dt = m_engine.GetFrameLength();
    const bool fineControlMode = m_engine.IsKeyDown(ysKey::Code::Space);

    const int mouseWheel = m_engine.GetMouseWheel();
    const int mouseWheelDelta = mouseWheel - m_lastMouseWheel;
    m_lastMouseWheel = mouseWheel;

    bool fineControlInUse = false;
    if (m_engine.IsKeyDown(ysKey::Code::Z)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.volume = clamp(audioParams.volume + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[Z] - Set volume to " + std::to_string(audioParams.volume));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::X)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.convolution = clamp(audioParams.convolution + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[X] - Set convolution level to " + std::to_string(audioParams.convolution));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::C)) {
        const double rate = fineControlMode
            ? 0.00001
            : 0.001;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.dF_F_mix = clamp(audioParams.dF_F_mix + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[C] - Set high freq. gain to " + std::to_string(audioParams.dF_F_mix));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::V)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.airNoise = clamp(audioParams.airNoise + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[V] - Set low freq. noise to " + std::to_string(audioParams.airNoise));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::B)) {
        const double rate = fineControlMode
            ? 0.001
            : 0.01;

        Synthesizer::AudioParameters audioParams = m_simulator->synthesizer().getAudioParameters();
        audioParams.inputSampleNoise = clamp(audioParams.inputSampleNoise + mouseWheelDelta * rate * dt);

        m_simulator->synthesizer().setAudioParameters(audioParams);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[B] - Set high freq. noise to " + std::to_string(audioParams.inputSampleNoise));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::N)) {
        const double rate = fineControlMode
            ? 10.0
            : 100.0;

        const double newSimulationFrequency = clamp(
            m_simulator->getSimulationFrequency() + mouseWheelDelta * rate * dt,
            400.0, 400000.0);

        m_simulator->setSimulationFrequency(newSimulationFrequency);
        fineControlInUse = true;

        m_infoCluster->setLogMessage("[N] - Set simulation freq to " + std::to_string(m_simulator->getSimulationFrequency()));
    }
    else if (m_engine.IsKeyDown(ysKey::Code::G) && m_simulator->m_dyno.m_hold) {
        if (mouseWheelDelta > 0) {
            m_dynoSpeed += m_iceEngine->getDynoHoldStep();
        }
        else if (mouseWheelDelta < 0) {
            m_dynoSpeed -= m_iceEngine->getDynoHoldStep();
        }

        m_dynoSpeed = clamp(m_dynoSpeed, m_iceEngine->getDynoMinSpeed(), m_iceEngine->getDynoMaxSpeed());

        m_infoCluster->setLogMessage("[G] - Set dyno speed to " + std::to_string(units::toRpm(m_dynoSpeed)));
        fineControlInUse = true;
    }

    const double prevTargetThrottle = m_targetSpeedSetting;
    m_targetSpeedSetting = fineControlMode ? m_targetSpeedSetting : 0.0;
    if (m_engine.IsKeyDown(ysKey::Code::Q)) {
        m_targetSpeedSetting = 0.01;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::W)) {
        m_targetSpeedSetting = 0.1;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::E)) {
        m_targetSpeedSetting = 0.2;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::R)) {
        m_targetSpeedSetting = 1.0;
    }
    else if (fineControlMode && !fineControlInUse) {
        m_targetSpeedSetting = clamp(m_targetSpeedSetting + mouseWheelDelta * 0.0001);
    }

    if (prevTargetThrottle != m_targetSpeedSetting) {
        m_infoCluster->setLogMessage("Speed control set to " + std::to_string(m_targetSpeedSetting));
    }

    m_speedSetting = m_targetSpeedSetting * 0.5 + 0.5 * m_speedSetting;

    m_iceEngine->setSpeedControl(m_speedSetting);
    if (m_engine.ProcessKeyDown(ysKey::Code::M)) {
        const int currentLayer = getViewParameters().Layer0;
        if (currentLayer + 1 < m_iceEngine->getMaxDepth()) {
            setViewLayer(currentLayer + 1);
        }

        m_infoCluster->setLogMessage("[M] - Set render layer to " + std::to_string(getViewParameters().Layer0));
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::OEM_Comma)) {
        if (getViewParameters().Layer0 - 1 >= 0)
            setViewLayer(getViewParameters().Layer0 - 1);

        m_infoCluster->setLogMessage("[,] - Set render layer to " + std::to_string(getViewParameters().Layer0));
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::D)) {
        m_simulator->m_dyno.m_enabled = !m_simulator->m_dyno.m_enabled;

        const std::string msg = m_simulator->m_dyno.m_enabled
            ? "DYNOMOMETER ENABLED"
            : "DYNOMOMETER DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::H)) {
        m_simulator->m_dyno.m_hold = !m_simulator->m_dyno.m_hold;

        const std::string msg = m_simulator->m_dyno.m_hold
            ? m_simulator->m_dyno.m_enabled ? "HOLD ENABLED" : "HOLD ON STANDBY [ENABLE DYNO. FOR HOLD]"
            : "HOLD DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_simulator->m_dyno.m_enabled) {
        if (!m_simulator->m_dyno.m_hold) {
            if (m_simulator->getFilteredDynoTorque() > units::torque(1.0, units::ft_lb)) {
                m_dynoSpeed += units::rpm(500) * dt;
            }
            else {
                m_dynoSpeed *= (1 / (1 + dt));
            }

            const double sweepLimit = (std::min)(
                m_iceEngine->getRedline(), m_iceEngine->getDynoMaxSpeed());
            if (m_dynoSpeed > sweepLimit) {
                m_simulator->m_dyno.m_enabled = false;
                m_dynoSpeed = units::rpm(0);
                m_infoCluster->setLogMessage("Dyno sweep completed");
            }
        }
    }
    else {
        if (!m_simulator->m_dyno.m_hold) {
            m_dynoSpeed = units::rpm(0);
        }
    }

    m_dynoSpeed = clamp(m_dynoSpeed, m_iceEngine->getDynoMinSpeed(), m_iceEngine->getDynoMaxSpeed());
    m_simulator->m_dyno.m_rotationSpeed = m_dynoSpeed;

    const bool prevStarterEnabled = m_simulator->m_starterMotor.m_enabled;
    if (m_engine.IsKeyDown(ysKey::Code::S) || m_diagnosticMode) {
        m_simulator->m_starterMotor.m_enabled = true;
    }
    else {
        m_simulator->m_starterMotor.m_enabled = false;
    }

    if (prevStarterEnabled != m_simulator->m_starterMotor.m_enabled) {
        const std::string msg = m_simulator->m_starterMotor.m_enabled
            ? "STARTER ENABLED"
            : "STARTER DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::A)) {
        m_simulator->getEngine()->getIgnitionModule()->m_enabled =
            !m_simulator->getEngine()->getIgnitionModule()->m_enabled;

        const std::string msg = m_simulator->getEngine()->getIgnitionModule()->m_enabled
            ? "IGNITION ENABLED"
            : "IGNITION DISABLED";
        m_infoCluster->setLogMessage(msg);
    }

    if (m_engine.ProcessKeyDown(ysKey::Code::Up)) {
        m_simulator->getTransmission()->changeGear(m_simulator->getTransmission()->getGear() + 1);

        m_infoCluster->setLogMessage(
            "UPSHIFTED TO " + std::to_string(m_simulator->getTransmission()->getGear() + 1));
    }
    else if (m_engine.ProcessKeyDown(ysKey::Code::Down)) {
        m_simulator->getTransmission()->changeGear(m_simulator->getTransmission()->getGear() - 1);

        if (m_simulator->getTransmission()->getGear() != -1) {
            m_infoCluster->setLogMessage(
                "DOWNSHIFTED TO " + std::to_string(m_simulator->getTransmission()->getGear() + 1));
        }
        else {
            m_infoCluster->setLogMessage("SHIFTED TO NEUTRAL");
        }
    }

    if (m_engine.IsKeyDown(ysKey::Code::T)) {
        m_targetClutchPressure -= 0.2 * dt;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::U)) {
        m_targetClutchPressure += 0.2 * dt;
    }
    else if (m_engine.IsKeyDown(ysKey::Code::Shift)) {
        m_targetClutchPressure = 0.0;
        m_infoCluster->setLogMessage("CLUTCH DEPRESSED");
    }
    else if (!m_engine.IsKeyDown(ysKey::Code::Y)) {
        m_targetClutchPressure = 1.0;
    }

    m_targetClutchPressure = clamp(m_targetClutchPressure);

    double clutchRC = 0.001;
    if (m_engine.IsKeyDown(ysKey::Code::Space)) {
        clutchRC = 1.0;
    }

    const double clutch_s = dt / (dt + clutchRC);
    m_clutchPressure = m_clutchPressure * (1 - clutch_s) + m_targetClutchPressure * clutch_s;
    m_simulator->getTransmission()->setClutchPressure(m_clutchPressure);
}

void EngineSimApplication::renderScene() {
    getShaders()->ResetBaseColor();
    getShaders()->SetObjectTransform(ysMath::LoadIdentity());

    m_textRenderer.SetColor(ysColor::linearToSrgb(m_foreground));
    m_shaders.SetClearColor(ysColor::linearToSrgb(m_shadow));

    const int screenWidth = m_engine.GetGameWindow()->GetGameWidth();
    const int screenHeight = m_engine.GetGameWindow()->GetGameHeight();
    const float aspectRatio = screenWidth / (float)screenHeight;

    const Point cameraPos = m_engineView->getCameraPosition();
    m_shaders.m_cameraPosition = ysMath::LoadVector(cameraPos.x, cameraPos.y);

    m_shaders.CalculateUiCamera(screenWidth, screenHeight);

    if (m_screen == 0) {
        Bounds windowBounds((float)screenWidth, (float)screenHeight, { 0, (float)screenHeight });
        Grid grid;
        grid.v_cells = 2;
        grid.h_cells = 3;
        Grid grid3x3;
        grid3x3.v_cells = 3;
        grid3x3.h_cells = 3;
        m_engineView->setDrawFrame(true);
        m_engineView->setBounds(grid.get(windowBounds, 1, 0, 1, 1));
        m_engineView->setLocalPosition({ 0, 0 });

        m_rightGaugeCluster->m_bounds = grid.get(windowBounds, 2, 0, 1, 2);
        m_oscCluster->m_bounds = grid.get(windowBounds, 1, 1);
        m_performanceCluster->m_bounds = grid3x3.get(windowBounds, 0, 1);
        m_loadSimulationCluster->m_bounds = grid3x3.get(windowBounds, 0, 2);

        Grid grid1x3;
        grid1x3.v_cells = 3;
        grid1x3.h_cells = 1;
        m_mixerCluster->m_bounds = grid1x3.get(grid3x3.get(windowBounds, 0, 0), 0, 2);
        m_infoCluster->m_bounds = grid1x3.get(grid3x3.get(windowBounds, 0, 0), 0, 0, 1, 2);

        m_engineView->setVisible(true);
        m_rightGaugeCluster->setVisible(true);
        m_oscCluster->setVisible(true);
        m_performanceCluster->setVisible(true);
        m_loadSimulationCluster->setVisible(true);
        m_mixerCluster->setVisible(true);
        m_infoCluster->setVisible(true);

        m_oscCluster->activate();
    }
    else if (m_screen == 1) {
        Bounds windowBounds((float)screenWidth, (float)screenHeight, { 0, (float)screenHeight });
        m_engineView->setDrawFrame(false);
        m_engineView->setBounds(windowBounds);
        m_engineView->setLocalPosition({ 0, 0 });
        m_engineView->activate();

        m_engineView->setVisible(true);
        m_rightGaugeCluster->setVisible(false);
        m_oscCluster->setVisible(false);
        m_performanceCluster->setVisible(false);
        m_loadSimulationCluster->setVisible(false);
        m_mixerCluster->setVisible(false);
        m_infoCluster->setVisible(false);
    }
    else if (m_screen == 2) {
        Bounds windowBounds((float)screenWidth, (float)screenHeight, { 0, (float)screenHeight });
        Grid grid;
        grid.v_cells = 1;
        grid.h_cells = 3;
        m_engineView->setDrawFrame(true);
        m_engineView->setBounds(grid.get(windowBounds, 0, 0, 2, 1));
        m_engineView->setLocalPosition({ 0, 0 });
        m_engineView->activate();

        m_rightGaugeCluster->m_bounds = grid.get(windowBounds, 2, 0, 1, 1);

        m_engineView->setVisible(true);
        m_rightGaugeCluster->setVisible(true);
        m_oscCluster->setVisible(false);
        m_performanceCluster->setVisible(false);
        m_loadSimulationCluster->setVisible(false);
        m_mixerCluster->setVisible(false);
        m_infoCluster->setVisible(false);
    }

    const float cameraAspectRatio =
        m_engineView->m_bounds.width() / m_engineView->m_bounds.height();
    m_engine.GetDevice()->ResizeRenderTarget(
        m_mainRenderTarget,
        m_engineView->m_bounds.width(),
        m_engineView->m_bounds.height(),
        m_engineView->m_bounds.width(),
        m_engineView->m_bounds.height()
    );
    m_engine.GetDevice()->RepositionRenderTarget(
        m_mainRenderTarget,
        m_engineView->m_bounds.getPosition(Bounds::tl).x,
        screenHeight - m_engineView->m_bounds.getPosition(Bounds::tl).y
    );
    m_shaders.CalculateCamera(
        cameraAspectRatio * m_displayHeight / m_engineView->m_zoom,
        m_displayHeight / m_engineView->m_zoom,
        m_engineView->m_bounds,
        m_screenWidth,
        m_screenHeight,
        m_displayAngle);

    m_geometryGenerator.reset();

    render();

    m_engine.GetDevice()->EditBufferDataRange(
        m_geometryVertexBuffer,
        (char *)m_geometryGenerator.getVertexData(),
        sizeof(dbasic::Vertex) * m_geometryGenerator.getCurrentVertexCount(),
        0);

    m_engine.GetDevice()->EditBufferDataRange(
        m_geometryIndexBuffer,
        (char *)m_geometryGenerator.getIndexData(),
        sizeof(unsigned short) * m_geometryGenerator.getCurrentIndexCount(),
        0);
}

void EngineSimApplication::refreshUserInterface() {
    m_uiManager.destroy();
    m_uiManager.initialize(this);

    m_engineView = m_uiManager.getRoot()->addElement<EngineView>();
    m_rightGaugeCluster = m_uiManager.getRoot()->addElement<RightGaugeCluster>();
    m_oscCluster = m_uiManager.getRoot()->addElement<OscilloscopeCluster>();
    m_performanceCluster = m_uiManager.getRoot()->addElement<PerformanceCluster>();
    m_loadSimulationCluster = m_uiManager.getRoot()->addElement<LoadSimulationCluster>();
    m_mixerCluster = m_uiManager.getRoot()->addElement<MixerCluster>();
    m_infoCluster = m_uiManager.getRoot()->addElement<InfoCluster>();

    m_infoCluster->setEngine(m_iceEngine);
    m_rightGaugeCluster->m_simulator = m_simulator;
    m_rightGaugeCluster->setEngine(m_iceEngine);
    m_oscCluster->setSimulator(m_simulator);
    if (m_iceEngine != nullptr) {
        m_oscCluster->setDynoMaxRange(units::toRpm(m_iceEngine->getRedline()));
    }
    m_performanceCluster->setSimulator(m_simulator);
    m_loadSimulationCluster->setSimulator(m_simulator);
    m_mixerCluster->setSimulator(m_simulator);
}

void EngineSimApplication::startRecording() {
#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
    if (m_recording || !readyToRecord()) return;
    atg_dtv::Encoder::VideoSettings settings{};

    const std::filesystem::path outputDirectory("video_capture");
    std::error_code directoryError;
    std::filesystem::create_directories(outputDirectory, directoryError);
    if (directoryError) {
        std::ofstream log("error_log.log", std::ios::app);
        log << "Cannot create video capture directory: " << directoryError.message() << '\n';
        m_infoCluster->setLogMessage("Cannot create video_capture folder; see error_log.log");
        return;
    }
    settings.fname = (outputDirectory / "engine_sim_video_capture.mp4").string();
    settings.inputWidth = m_engine.GetScreenWidth();
    settings.inputHeight = m_engine.GetScreenHeight();
    settings.width = settings.inputWidth;
    settings.height = settings.inputHeight;
    settings.hardwareEncoding = true;
    settings.inputAlpha = true;
    settings.bitRate = 40000000;

    m_encoder.run(settings, 2);
    m_recording = true;
#else
    m_infoCluster->setLogMessage("Video recording is unavailable in this build");
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
}

void EngineSimApplication::updateScreenSizeStability() {
    m_screenResolution[m_screenResolutionIndex][0] = m_engine.GetScreenWidth();
    m_screenResolution[m_screenResolutionIndex][1] = m_engine.GetScreenHeight();

    m_screenResolutionIndex = (m_screenResolutionIndex + 1) % ScreenResolutionHistoryLength;
}

bool EngineSimApplication::readyToRecord() {
    const int w = m_screenResolution[0][0];
    const int h = m_screenResolution[0][1];

    if (w <= 0 || h <= 0) return false;
    if ((w % 2) != 0 || (h % 2) != 0) return false;

    for (int i = 1; i < ScreenResolutionHistoryLength; ++i) {
        if (m_screenResolution[i][0] != w) return false;
        if (m_screenResolution[i][1] != h) return false;
    }

    return true;
}

void EngineSimApplication::stopRecording() {
    if (!m_recording) return;
    m_recording = false;

#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
    m_encoder.commit();
    m_encoder.stop();
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
}

void EngineSimApplication::recordFrame() {
#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
    if (m_encoder.getError() != atg_dtv::Encoder::Error::None) {
        stopRecording();
        m_infoCluster->setLogMessage("Video encoder failed; recording stopped");
        return;
    }
    atg_dtv::Frame *frame = m_encoder.newFrame(false);
    if (frame == nullptr) return;
    if (m_engine.GetDevice()->ReadRenderTarget(m_engine.GetScreenRenderTarget(), frame->m_rgb) != ysError::None) {
        stopRecording();
        m_infoCluster->setLogMessage("Frame capture failed; recording stopped");
        return;
    }
    m_encoder.submitFrame();
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
}
