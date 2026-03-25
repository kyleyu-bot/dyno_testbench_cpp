set(SOEM_INCLUDES
    ${PROJECT_SOURCE_DIR}/include
    ${soem_SOURCE_DIR}/soem
    ${soem_SOURCE_DIR}/osal
    ${soem_SOURCE_DIR}/osal/linux
    ${soem_SOURCE_DIR}/oshw/linux
)

# ── drive_simple_speed_test1 ──────────────────────────────────────────────────

add_executable(drive_simple_speed_test1 drive_simple_speed_test1.cpp)
target_include_directories(drive_simple_speed_test1 PRIVATE ${SOEM_INCLUDES})
target_link_libraries(drive_simple_speed_test1 PRIVATE ethercat_core soem pthread rt)
target_compile_options(drive_simple_speed_test1 PRIVATE -Wall -Wextra)

# ── elm3002_data_read ─────────────────────────────────────────────────────────

add_executable(elm3002_data_read elm3002_data_read.cpp)
target_include_directories(elm3002_data_read PRIVATE ${SOEM_INCLUDES})
target_link_libraries(elm3002_data_read PRIVATE ethercat_core soem pthread rt)
target_compile_options(elm3002_data_read PRIVATE -Wall -Wextra)

# ── elm3002_pdo_check ─────────────────────────────────────────────────────────

add_executable(elm3002_pdo_check elm3002_pdo_check.cpp)
target_include_directories(elm3002_pdo_check PRIVATE ${SOEM_INCLUDES})
target_link_libraries(elm3002_pdo_check PRIVATE ethercat_core soem pthread rt)
target_compile_options(elm3002_pdo_check PRIVATE -Wall -Wextra)

# ── el5032_data_read ──────────────────────────────────────────────────────────

add_executable(el5032_data_read el5032_data_read.cpp)
target_include_directories(el5032_data_read PRIVATE ${SOEM_INCLUDES})
target_link_libraries(el5032_data_read PRIVATE ethercat_core soem pthread rt)
target_compile_options(el5032_data_read PRIVATE -Wall -Wextra)

# ── el2004_output_hold ────────────────────────────────────────────────────────

add_executable(el2004_output_hold el2004_output_hold.cpp)
target_include_directories(el2004_output_hold PRIVATE ${SOEM_INCLUDES})
target_link_libraries(el2004_output_hold PRIVATE ethercat_core soem pthread rt)
target_compile_options(el2004_output_hold PRIVATE -Wall -Wextra)

# ── drive_simple_speed_test2 ──────────────────────────────────────────────────

add_executable(drive_simple_speed_test2 drive_simple_speed_test2.cpp)
target_include_directories(drive_simple_speed_test2 PRIVATE ${SOEM_INCLUDES})
target_link_libraries(drive_simple_speed_test2 PRIVATE ethercat_core soem pthread rt)
target_compile_options(drive_simple_speed_test2 PRIVATE -Wall -Wextra)

# ── dyno2_integrated_test1 ────────────────────────────────────────────────────

add_executable(dyno2_integrated_test1 dyno2_integrated_test1.cpp)
target_include_directories(dyno2_integrated_test1 PRIVATE ${SOEM_INCLUDES})
target_link_libraries(dyno2_integrated_test1 PRIVATE ethercat_core soem pthread rt)
target_compile_options(dyno2_integrated_test1 PRIVATE -Wall -Wextra)

# ── dyno2_integrated_test2 ────────────────────────────────────────────────────

add_executable(dyno2_integrated_test2 dyno2_integrated_test2.cpp)
target_include_directories(dyno2_integrated_test2 PRIVATE ${SOEM_INCLUDES})
target_link_libraries(dyno2_integrated_test2 PRIVATE ethercat_core soem pthread rt)
target_compile_options(dyno2_integrated_test2 PRIVATE -Wall -Wextra)
