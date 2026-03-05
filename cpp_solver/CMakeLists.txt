cmake_minimum_required(VERSION 3.12)
project(mpc_solver_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)

# Include Homebrew directories typically found on Apple Silicon
include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)

# The CasADi generated C file
add_library(mpc_solver_c STATIC src/mpc_solver.c)
target_link_libraries(mpc_solver_c PUBLIC ipopt)

# The Pybind11 module
pybind11_add_module(mpc_solver_py src/bindings.cpp)
target_link_libraries(mpc_solver_py PRIVATE mpc_solver_c)

# Set the output directory to be in the python path
set_target_properties(mpc_solver_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../uav_mpc/controllers
)
