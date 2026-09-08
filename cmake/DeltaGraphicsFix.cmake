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
