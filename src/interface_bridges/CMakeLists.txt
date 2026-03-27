set(BRIDGE_INCLUDES
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/interface_bridges
    ${soem_SOURCE_DIR}/soem
    ${soem_SOURCE_DIR}/osal
    ${soem_SOURCE_DIR}/osal/linux
    ${soem_SOURCE_DIR}/oshw/linux
)

# ── bridge_udp ────────────────────────────────────────────────────────────────
# Shared C++ EtherCAT + UDP backend used by all three interface options.
# Run this first, then launch whichever interface script you want:
#   ros2/dyno_ros2_node.py   — ROS2 topics
#   qt/dyno_qt_bridge.py     — PySide6 live dashboard
#   udp/dyno_plotly_dash.py  — browser dashboard (Plotly Dash)

add_executable(bridge_udp udp/bridge_udp.cpp)
target_include_directories(bridge_udp PRIVATE ${BRIDGE_INCLUDES})
target_link_libraries(bridge_udp PRIVATE ethercat_core soem pthread rt)
target_compile_options(bridge_udp PRIVATE -Wall -Wextra)
