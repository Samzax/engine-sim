find_package(Threads REQUIRED)
add_executable(engine-sim-audio-test
    test/audio_regression_tests.cpp
    src/synthesizer.cpp
    src/convolution_filter.cpp
    src/filter.cpp
    src/jitter_filter.cpp
    src/leveling_filter.cpp
    src/low_pass_filter.cpp
    src/derivative_filter.cpp)
target_sources(engine-sim-audio-test PRIVATE src/wave_reader.cpp)
target_link_libraries(engine-sim-audio-test PRIVATE Threads::Threads)
add_test(NAME audio-regressions COMMAND engine-sim-audio-test)
set_tests_properties(audio-regressions PROPERTIES TIMEOUT 60)
