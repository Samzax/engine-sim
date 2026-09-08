# The optional Direct3D debug layer is not installed on many Windows PCs.
# Retry only its specific missing-component error, preserving other failures.
if (NOT WIN32)
    return()
endif()
get_target_property(_graphics_dir delta-core SOURCE_DIR)
get_target_property(_graphics_sources delta-core SOURCES)
set(_graphics_source "${_graphics_dir}/src/yds_d3d11_device.cpp")
file(READ "${_graphics_source}" _graphics_code)
set(_creation_end [=[        &highestFeatureLevel,
        &m_deviceContext);
]=])
set(_creation_retry [=[        &highestFeatureLevel,
        &m_deviceContext);

    if (result == DXGI_ERROR_SDK_COMPONENT_MISSING
        && (deviceCreationFlags & D3D11_CREATE_DEVICE_DEBUG) != 0) {
        OutputDebugStringA("Engine Sim: Direct3D debug layer unavailable; using the standard runtime.\n");
        result = D3D11CreateDevice(
            nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
            deviceCreationFlags & ~D3D11_CREATE_DEVICE_DEBUG,
            nullptr, 0, D3D11_SDK_VERSION,
            &m_device, &highestFeatureLevel, &m_deviceContext);
    }
]=])
string(FIND "${_graphics_code}" "${_creation_end}" _graphics_match)
if (_graphics_match EQUAL -1)
    message(FATAL_ERROR "Delta graphics source changed; review cmake/DeltaGraphicsFix.cmake")
endif()
string(REPLACE "${_creation_end}" "${_creation_retry}" _graphics_code "${_graphics_code}")
set(_present_call "    context->m_swapChain->Present(1, 0);")
set(_checked_present [=[    const HRESULT presentResult = context->m_swapChain->Present(1, 0);
    if (FAILED(presentResult)) {
        std::ofstream log("error_log.log", std::ios::app);
        log << "Direct3D Present failed: HRESULT 0x" << std::hex
            << static_cast<unsigned long>(presentResult);
        if (presentResult == DXGI_ERROR_DEVICE_REMOVED || presentResult == DXGI_ERROR_DEVICE_RESET) {
            log << "; device removal reason 0x"
                << static_cast<unsigned long>(m_device->GetDeviceRemovedReason());
        }
        log << '\n';
        return YDS_ERROR_RETURN(ysError::ApiError);
    }]=])
string(FIND "${_graphics_code}" "${_present_call}" _present_match)
if (_present_match EQUAL -1)
    message(FATAL_ERROR "Delta Present implementation changed; review cmake/DeltaGraphicsFix.cmake")
endif()
string(REPLACE "${_present_call}" "${_checked_present}" _graphics_code "${_graphics_code}")
# ReadRenderTarget must release its first allocation if the second fails, and
# must never copy mapped data when Map failed.
set(_staging_failure [=[    if (FAILED(result) || stagingTexture == nullptr) {
        return YDS_ERROR_RETURN(ysError::ApiError);
    }]=])
set(_staging_cleanup [=[    if (FAILED(result) || stagingTexture == nullptr) {
        resolveTexture->Release();
        return YDS_ERROR_RETURN(ysError::ApiError);
    }]=])
set(_map_end [=[        &mappedResource);

    for (int i = 0; i < src->GetHeight(); ++i) {]=])
set(_checked_map [=[        &mappedResource);

    if (FAILED(result)) {
        stagingTexture->Release();
        resolveTexture->Release();
        return YDS_ERROR_RETURN(ysError::ApiError);
    }

    for (int i = 0; i < src->GetHeight(); ++i) {]=])
foreach(_readback_anchor IN ITEMS _staging_failure _map_end)
    string(FIND "${_graphics_code}" "${${_readback_anchor}}" _readback_match)
    if (_readback_match EQUAL -1)
        message(FATAL_ERROR "Delta readback implementation changed; review cmake/DeltaGraphicsFix.cmake")
    endif()
endforeach()
string(REPLACE "${_staging_failure}" "${_staging_cleanup}" _graphics_code "${_graphics_code}")
string(REPLACE "${_map_end}" "${_checked_map}" _graphics_code "${_graphics_code}")
string(PREPEND _graphics_code "#include <fstream>\n")
string(REPLACE "\"../include/" "\"${_graphics_dir}/include/" _graphics_code "${_graphics_code}")
set(_graphics_patched "${PROJECT_BINARY_DIR}/dependency-fixes/yds_d3d11_device.cpp")
file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/dependency-fixes")
set(_graphics_previous "")
if (EXISTS "${_graphics_patched}")
    file(READ "${_graphics_patched}" _graphics_previous)
endif()
if (NOT _graphics_previous STREQUAL _graphics_code)
    file(WRITE "${_graphics_patched}" "${_graphics_code}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_graphics_source}")
set(_graphics_found FALSE)
foreach(_source IN LISTS _graphics_sources)
    if (_source STREQUAL "src/yds_d3d11_device.cpp" OR _source STREQUAL "${_graphics_source}")
        list(REMOVE_ITEM _graphics_sources "${_source}")
        set(_graphics_found TRUE)
    endif()
endforeach()
if (NOT _graphics_found)
    message(FATAL_ERROR "Delta graphics target changed; review cmake/DeltaGraphicsFix.cmake")
endif()
set_property(TARGET delta-core PROPERTY SOURCES "${_graphics_sources}")
target_sources(delta-core PRIVATE "${_graphics_patched}")

# Engine Sim's entry point selects D3D11. References to the other factories
# otherwise make the static linker import their runtime DLLs at process startup.
option(ENGINE_SIM_D3D11_ONLY "Exclude unused graphics factories from Engine Sim's Delta build" ON)
if (ENGINE_SIM_D3D11_ONLY)
    set(_factory_source "${_graphics_dir}/src/yds_device.cpp")
    file(READ "${_factory_source}" _factory_code)
    foreach(_backend ysD3D10Device ysOpenGLDevice ysVulkanDevice)
        set(_factory_old "*newDevice = new ${_backend};")
        string(FIND "${_factory_code}" "${_factory_old}" _factory_match)
        if (_factory_match EQUAL -1)
            message(FATAL_ERROR "Delta device factory changed; review cmake/DeltaGraphicsFix.cmake")
        endif()
        string(REPLACE "${_factory_old}" "return YDS_ERROR_RETURN_STATIC(ysError::NotImplemented);"
            _factory_code "${_factory_code}")
    endforeach()
    string(REPLACE "\"../include/" "\"${_graphics_dir}/include/" _factory_code "${_factory_code}")
    set(_factory_patched "${PROJECT_BINARY_DIR}/dependency-fixes/yds_device.cpp")
    set(_factory_previous "")
    if (EXISTS "${_factory_patched}")
        file(READ "${_factory_patched}" _factory_previous)
    endif()
    if (NOT _factory_previous STREQUAL _factory_code)
        file(WRITE "${_factory_patched}" "${_factory_code}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_factory_source}")
    get_target_property(_factory_sources delta-core SOURCES)
    set(_factory_found FALSE)
    foreach(_source IN LISTS _factory_sources)
        if (_source STREQUAL "src/yds_device.cpp" OR _source STREQUAL "${_factory_source}")
            list(REMOVE_ITEM _factory_sources "${_source}")
            set(_factory_found TRUE)
        endif()
    endforeach()
    if (NOT _factory_found)
        message(FATAL_ERROR "Delta device target changed; review cmake/DeltaGraphicsFix.cmake")
    endif()
    set_property(TARGET delta-core PROPERTY SOURCES "${_factory_sources}")
    target_sources(delta-core PRIVATE "${_factory_patched}")
endif()
