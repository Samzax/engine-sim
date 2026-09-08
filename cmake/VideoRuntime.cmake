# The FFmpeg import libraries do not carry their DLLs into the application.
get_filename_component(_ffmpeg_library_dir "${AVCODEC_LIBRARY}" DIRECTORY)
get_filename_component(_ffmpeg_default_bin "${_ffmpeg_library_dir}/../bin" ABSOLUTE)
set(ENGINE_SIM_FFMPEG_RUNTIME_DIR "${_ffmpeg_default_bin}" CACHE PATH
    "Directory containing the FFmpeg shared runtime DLLs to package")
file(GLOB _ffmpeg_dlls CONFIGURE_DEPENDS "${ENGINE_SIM_FFMPEG_RUNTIME_DIR}/*.dll")
file(GLOB _ffmpeg_codec_dlls "${ENGINE_SIM_FFMPEG_RUNTIME_DIR}/avcodec*.dll")
if (NOT _ffmpeg_codec_dlls)
    message(FATAL_ERROR "Set ENGINE_SIM_FFMPEG_RUNTIME_DIR to the FFmpeg shared build's bin directory")
endif()
find_file(ENGINE_SIM_FFMPEG_LICENSE NAMES LICENSE LICENSE.txt COPYING.GPLv3 COPYING.LGPLv3
    HINTS "${ENGINE_SIM_FFMPEG_RUNTIME_DIR}/.." "${ENGINE_SIM_FFMPEG_RUNTIME_DIR}/../doc")
if (NOT ENGINE_SIM_FFMPEG_LICENSE)
    message(FATAL_ERROR "Set ENGINE_SIM_FFMPEG_LICENSE to the license supplied with the FFmpeg shared build")
endif()
add_custom_command(TARGET engine-sim-app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_ffmpeg_dlls}
        "$<TARGET_FILE_DIR:engine-sim-app>"
    VERBATIM)
install(FILES ${_ffmpeg_dlls} DESTINATION .)
install(FILES "${ENGINE_SIM_FFMPEG_LICENSE}" DESTINATION licenses RENAME FFmpeg.txt)
if (EXISTS "${ENGINE_SIM_FFMPEG_RUNTIME_DIR}/../README.txt")
    install(FILES "${ENGINE_SIM_FFMPEG_RUNTIME_DIR}/../README.txt"
        DESTINATION licenses RENAME FFmpeg-build.txt)
endif()
