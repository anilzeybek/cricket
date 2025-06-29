cmake_minimum_required(VERSION 3.10)
project(pinocchio_codegen_example)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required packages
find_package(PkgConfig REQUIRED)

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)
set(INJA_USE_EMBEDDED_JSON OFF CACHE INTERNAL "")
set(INJA_BUILD_TESTS OFF CACHE INTERNAL "")
add_subdirectory(inja)

# Find Pinocchio
find_package(pinocchio REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(fmt REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem)

# Find CppAD and CppADCodeGen
find_package(cppad QUIET)
if(NOT cppad_FOUND)
    pkg_check_modules(cppad REQUIRED cppad)
endif()

find_package(cppadcg QUIET)
if(NOT cppadcg_FOUND)
    pkg_check_modules(cppadcg REQUIRED cppadcg)
endif()

# Create the executable
add_executable(fkcc_gen fkcc_gen.cc)

# Set include directories
target_include_directories(fkcc_gen PRIVATE
    ${pinocchio_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${cppad_INCLUDE_DIR}
    ${cppadcg_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(fkcc_gen
    pinocchio::pinocchio
    pinocchio::pinocchio_cppadcg
    ${Boost_LIBRARIES}
    ${cppad_LIBRARIES}
    ${cppadcg_LIBRARIES}
    pantor::inja
    nlohmann_json::nlohmann_json
    CGAL::CGAL
    fmt::fmt
)

# Add compiler definitions
target_compile_definitions(fkcc_gen PRIVATE
    ${pinocchio_CFLAGS_OTHER}
    PINOCCHIO_WITH_CPPADCG
)
