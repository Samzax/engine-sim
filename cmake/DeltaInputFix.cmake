# Clear held inputs on focus loss, before background key-up messages are discarded.
if (NOT WIN32)
    return()
endif()
get_target_property(_delta_dir delta-core SOURCE_DIR)
get_target_property(_delta_sources delta-core SOURCES)
set(_input_source "${_delta_dir}/src/yds_windows_window_system.cpp")
file(READ "${_input_source}" _input_code)
set(_old_deactivate "target->OnDeactivate();")
set(_new_deactivate [=[if (inputSystem != nullptr && !inputSystem->IsGlobalInputEnabled()) {
                auto *keyboards = inputSystem->GetKeyboardAggregator();
                for (int key = 0; key < static_cast<int>(ysKey::Code::Count); ++key) {
                    keyboards->SetKeyState(static_cast<ysKey::Code>(key),
                        ysKey::State::Up, ysKey::Variation::Undefined);
                }
                auto *mice = inputSystem->GetMouseAggregator();
                for (int mouse = 0; mouse < mice->GetMouseCount(); ++mouse) {
                    for (int button = 0; button < static_cast<int>(ysMouse::Button::Count); ++button) {
                        mice->GetMouse(mouse)->UpdateButton(static_cast<ysMouse::Button>(button),
                            ysMouse::ButtonState::Up);
                    }
                }
            }
            target->OnDeactivate();]=])
string(FIND "${_input_code}" "${_old_deactivate}" _match)
if (_match EQUAL -1)
    message(FATAL_ERROR "Delta input source changed; review or remove cmake/DeltaInputFix.cmake")
endif()
string(REPLACE "${_old_deactivate}" "${_new_deactivate}" _input_code "${_input_code}")
# Preserve the original source's header lookup when compiling its generated copy.
string(REPLACE "\"../include/" "\"${_delta_dir}/include/" _input_code "${_input_code}")
set(_patched_source "${PROJECT_BINARY_DIR}/dependency-fixes/yds_windows_window_system.cpp")
file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/dependency-fixes")
set(_previous_code "")
if (EXISTS "${_patched_source}")
    file(READ "${_patched_source}" _previous_code)
endif()
if (NOT _previous_code STREQUAL _input_code)
    file(WRITE "${_patched_source}" "${_input_code}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_input_source}")
set(_found_source FALSE)
foreach(_source IN LISTS _delta_sources)
    if (_source STREQUAL "src/yds_windows_window_system.cpp" OR _source STREQUAL "${_input_source}")
        list(REMOVE_ITEM _delta_sources "${_source}")
        set(_found_source TRUE)
    endif()
endforeach()
if (NOT _found_source)
    message(FATAL_ERROR "Delta input target changed; review cmake/DeltaInputFix.cmake")
endif()
set_property(TARGET delta-core PROPERTY SOURCES "${_delta_sources}")
target_sources(delta-core PRIVATE "${_patched_source}")
