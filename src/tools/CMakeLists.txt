add_executable(scan_soem scan_soem.cpp)

target_include_directories(scan_soem PRIVATE
    ${soem_SOURCE_DIR}/soem
    ${soem_SOURCE_DIR}/osal
    ${soem_SOURCE_DIR}/osal/linux
    ${soem_SOURCE_DIR}/oshw/linux
)

target_link_libraries(scan_soem PRIVATE soem pthread rt)

target_compile_options(scan_soem PRIVATE -Wall -Wextra)
