if (NOT WIN32)
    return()
endif()

# Development executables use the checkout's assets regardless of launch folder.
file(GENERATE OUTPUT "$<TARGET_FILE_DIR:engine-sim-app>/delta.conf"
    CONTENT "${PROJECT_SOURCE_DIR}/dependencies/submodules/delta-studio/engines/basic\n${PROJECT_SOURCE_DIR}/assets\n")

if (NOT ENGINE_SIM_D3D11_ONLY)
get_filename_component(_sdl_lib_dir "${SDL2_LIBRARY}" DIRECTORY)
get_filename_component(_image_lib_dir "${SDL2_IMAGE_LIBRARY}" DIRECTORY)
find_file(ENGINE_SIM_SDL_RUNTIME SDL2.dll HINTS "${_sdl_lib_dir}" "${SDL2_DIR}/lib/x64")
find_file(ENGINE_SIM_IMAGE_RUNTIME SDL2_image.dll HINTS "${_image_lib_dir}")
if (NOT ENGINE_SIM_SDL_RUNTIME OR NOT ENGINE_SIM_IMAGE_RUNTIME)
    message(FATAL_ERROR "SDL runtime DLLs are missing; run tools/setup-windows.ps1 or set ENGINE_SIM_*_RUNTIME")
endif()
add_custom_command(TARGET engine-sim-app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ENGINE_SIM_SDL_RUNTIME}" "${ENGINE_SIM_IMAGE_RUNTIME}"
        "$<TARGET_FILE_DIR:engine-sim-app>"
    VERBATIM)
endif()

set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME Runtime)
install(TARGETS engine-sim-app RUNTIME DESTINATION .)
if (TARGET engine-sim-headless)
    install(TARGETS engine-sim-headless RUNTIME DESTINATION .)
endif()
if (NOT ENGINE_SIM_D3D11_ONLY)
    install(FILES "${ENGINE_SIM_SDL_RUNTIME}" "${ENGINE_SIM_IMAGE_RUNTIME}" DESTINATION .)
    install(FILES "${SDL2_DIR}/COPYING.txt" DESTINATION licenses RENAME SDL2.txt)
    get_filename_component(_image_root "${_image_lib_dir}/../.." ABSOLUTE)
    install(FILES "${_image_root}/LICENSE.txt" DESTINATION licenses RENAME SDL2_image.txt)
endif()
install(DIRECTORY "${PROJECT_SOURCE_DIR}/assets" "${PROJECT_SOURCE_DIR}/es" DESTINATION .)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/dependencies/submodules/delta-studio/engines/basic/fonts"
    "${PROJECT_SOURCE_DIR}/dependencies/submodules/delta-studio/engines/basic/shaders" DESTINATION engine)
install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" DESTINATION .)
install(FILES "${PROJECT_SOURCE_DIR}/docs/portable-release.md" DESTINATION . RENAME README.md)
install(FILES "${PROJECT_SOURCE_DIR}/docs/headless.md" DESTINATION .)
foreach(_dependency delta-studio csv-io simple-2d-constraint-solver)
    install(FILES "${PROJECT_SOURCE_DIR}/dependencies/submodules/${_dependency}/LICENSE"
        DESTINATION licenses RENAME "${_dependency}.txt")
endforeach()
install(FILES "${PROJECT_SOURCE_DIR}/dependencies/discord/LICENSE" DESTINATION licenses RENAME discord.txt)

# Ship the redistributable release CRT beside the executable.
set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION .)
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT Runtime)
include(InstallRequiredSystemLibraries)
set(CPACK_GENERATOR ZIP)
set(CPACK_PACKAGE_NAME engine-sim)
set(CPACK_PACKAGE_VERSION 0.1.0)
set(CPACK_PACKAGE_FILE_NAME engine-sim-windows-x64)
set(CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};${PROJECT_NAME};Runtime;/")
include(CPack)
