#include "../dependencies/submodules/delta-studio/include/yds_ds8_audio_source.h"
#include <iostream>
#include <stdexcept>
#include <cstdint>
#include <cstring>

namespace {
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
struct Device {
    HWND window = nullptr;
    IDirectSound8 *sound = nullptr;
    ~Device() {
        if (sound) sound->Release();
        if (window) DestroyWindow(window);
    }
};
class Source : public ysDS8AudioSource {
public:
    void create(IDirectSound8 *device) {
        m_audioParameters.m_bitsPerSample = 16;
        m_audioParameters.m_channelCount = 1;
        m_audioParameters.m_sampleRate = 44100;
        m_bufferSize = 4096;
        WAVEFORMATEX format{};
        format.wFormatTag = WAVE_FORMAT_PCM;
        format.nChannels = 1;
        format.nSamplesPerSec = 44100;
        format.wBitsPerSample = 16;
        format.nBlockAlign = 2;
        format.nAvgBytesPerSec = 88200;
        DSBUFFERDESC description{};
        description.dwSize = sizeof(description);
        description.dwBufferBytes = 8192;
        description.lpwfxFormat = &format;
        require(SUCCEEDED(device->CreateSoundBuffer(&description, &m_buffer, nullptr)), "CreateSoundBuffer failed");
    }
    ~Source() { if (m_buffer) Destroy(); }
};
}

int main() {
    try {
        const HWND foreground = GetForegroundWindow();
        Device device;
        // No WS_VISIBLE, ShowWindow, activation or input injection. DirectSound
        // gets an owned hidden HWND solely for its cooperative-level contract.
        device.window = CreateWindowExW(WS_EX_NOACTIVATE | WS_EX_TOOLWINDOW, L"STATIC",
            L"Engine Sim audio diagnostic", WS_POPUP, 0, 0, 1, 1, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
        require(device.window != nullptr, "Hidden window creation failed");
        require(!IsWindowVisible(device.window), "Diagnostic window unexpectedly visible");
        require(SUCCEEDED(DirectSoundCreate8(nullptr, &device.sound, nullptr)), "No DirectSound device available");
        require(SUCCEEDED(device.sound->SetCooperativeLevel(device.window, DSSCL_NORMAL)), "SetCooperativeLevel failed");
        Source source;
        source.create(device.sound);
        void *whole = nullptr;
        SampleOffset wholeSize = 0;
        require(source.LockEntireBuffer(&whole, &wholeSize) == ysError::None, "Whole buffer lock failed");
        std::memset(whole, 0, wholeSize * sizeof(int16_t));
        require(source.UnlockBuffer(whole, wholeSize) == ysError::None, "Whole buffer unlock failed");
        for (SampleOffset offset : {SampleOffset(0), SampleOffset(4090)}) {
            void *first = nullptr, *second = nullptr;
            SampleOffset firstSize = 0, secondSize = 0;
            require(source.LockBufferSegment(offset, 32, &first, &firstSize, &second, &secondSize) == ysError::None,
                "Segment lock failed");
            require(firstSize + secondSize == 32, "Unexpected segment sizes");
            for (SampleOffset i = 0; i < firstSize; ++i) static_cast<int16_t *>(first)[i] = static_cast<int16_t>(i + 1);
            for (SampleOffset i = 0; i < secondSize; ++i) static_cast<int16_t *>(second)[i] = static_cast<int16_t>(firstSize + i + 1);
            require(source.UnlockBufferSegments(first, firstSize, second, secondSize) == ysError::None, "Segment unlock failed");
            require(source.LockEntireBuffer(&whole, &wholeSize) == ysError::None, "Readback lock failed");
            bool matches = true;
            for (SampleOffset i = 0; i < 32; ++i) matches &= static_cast<int16_t *>(whole)[(offset + i) % 4096] == i + 1;
            require(source.UnlockBuffer(whole, wholeSize) == ysError::None, "Readback unlock failed");
            require(matches, "Audio buffer readback mismatch");
        }
        std::cout << "DirectSound whole/segmented/wrapped buffer upload and readback passed; no playback requested.\n";
        std::cout << "Foreground window " << (foreground == GetForegroundWindow() ? "unchanged" : "changed during check") << ".\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
