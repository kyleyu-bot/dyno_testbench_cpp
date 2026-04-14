cmake_minimum_required(VERSION 3.20)
project(dyno_testbench_cpp
    VERSION 0.1.0
    DESCRIPTION "Dyno testbench EtherCAT control stack (C++)"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
endif()

# ── External dependencies via FetchContent ────────────────────────────────────
include(FetchContent)

# ─────────────────────────────────────────────────────────────────────────────
# SOEM – Simple Open EtherCAT Master
#
# SOEM_GIT_TAG controls which version is fetched.  Two strategies:
#
#   Strategy A — branch/tag name (easy to start, not fully reproducible):
#       set(SOEM_GIT_TAG    "master")
#       set(SOEM_GIT_SHALLOW ON)
#
#   Strategy B — pinned full 40-char commit SHA (recommended for production):
#       1. Browse https://github.com/OpenEtherCATsociety/SOEM/commits/master
#       2. Copy the full SHA of the desired commit.
#       3. Paste it as SOEM_GIT_TAG and set SOEM_GIT_SHALLOW to OFF.
#          Shallow clones cannot resolve raw commit SHAs.
#
#   Example pin (replace with actual SHA from the link above):
#       set(SOEM_GIT_TAG    "a5b9c8f2c9e54d8374a3a2eb5c82cb0ef0e15ea2")
#       set(SOEM_GIT_SHALLOW OFF)
#
#   To find the current HEAD SHA without cloning:
#       git ls-remote https://github.com/OpenEtherCATsociety/SOEM.git HEAD
# ─────────────────────────────────────────────────────────────────────────────
set(SOEM_GIT_TAG     "v1.4.0" CACHE STRING "SOEM git branch, tag, or full commit SHA")
set(SOEM_GIT_SHALLOW ON       CACHE BOOL   "Shallow clone — must be OFF when SOEM_GIT_TAG is a commit SHA")

FetchContent_Declare(
    soem
    GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM.git
    GIT_TAG        ${SOEM_GIT_TAG}
    GIT_SHALLOW    ${SOEM_GIT_SHALLOW}
)
FetchContent_MakeAvailable(soem)

# SOEM v1.4.0 has a strncpy pattern in oshw/linux/oshw.c that GCC 11 flags as
# -Wstringop-truncation.  SOEM's own CMakeLists sets -Werror, so this becomes a
# build error.  Suppress it only for the soem target.
target_compile_options(soem PRIVATE -Wno-stringop-truncation)

# ─────────────────────────────────────────────────────────────────────────────
# nlohmann/json – header-only JSON library for topology config loading
# ─────────────────────────────────────────────────────────────────────────────
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    ON
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

# ── Project subdirectories ────────────────────────────────────────────────────
add_subdirectory(src/ethercat_core)
add_subdirectory(src/tools)
add_subdirectory(src/ecat_functional_tests)
add_subdirectory(src/interface_bridges)
