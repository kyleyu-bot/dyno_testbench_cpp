add_library(testbench_utils STATIC
    function_generator.cpp
)

target_include_directories(testbench_utils PUBLIC
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_options(testbench_utils PRIVATE -Wall -Wextra)
