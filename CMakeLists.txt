cmake_minimum_required(VERSION 3.20)

project(nmpc_bench LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(NMPC_BENCH_COMMON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include")

set(DEFAULT_CASADI_ROOT "$ENV{HOME}/.local/lib/python3.12/site-packages/casadi")
if(EXISTS "${DEFAULT_CASADI_ROOT}/include/casadi/casadi.hpp")
    set(CASADI_ROOT "${DEFAULT_CASADI_ROOT}" CACHE PATH "Path to a CasADi installation root containing include/ and libcasadi.so")
else()
    set(CASADI_ROOT "" CACHE PATH "Path to a CasADi installation root containing include/ and libcasadi.so")
endif()

set(MINISOLVER_CASE "pendulum_on_cart" CACHE STRING "Official case to build for the MiniSolver native target")
set_property(CACHE MINISOLVER_CASE PROPERTY STRINGS
    pendulum_on_cart
    race_cars
    quadrotor_nav
    chain_mass
)

set(ACADOS_OUTPUT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/out/acados" CACHE PATH "Path to exported acados official-case assets")

set(DEFAULT_MINISOLVER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MiniSolver")
if(EXISTS "${DEFAULT_MINISOLVER_SOURCE_DIR}/CMakeLists.txt")
    set(MINISOLVER_SOURCE_DIR "${DEFAULT_MINISOLVER_SOURCE_DIR}" CACHE PATH "Path to a MiniSolver checkout")
else()
    set(MINISOLVER_SOURCE_DIR "" CACHE PATH "Path to a MiniSolver checkout")
endif()

if(MINISOLVER_SOURCE_DIR)
    if(NOT EXISTS "${MINISOLVER_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR "MINISOLVER_SOURCE_DIR does not point to a MiniSolver checkout: ${MINISOLVER_SOURCE_DIR}")
    endif()

    add_subdirectory("${MINISOLVER_SOURCE_DIR}" _deps/minisolver EXCLUDE_FROM_ALL)

    set(MINISOLVER_BENCH_SOURCES
        targets/minisolver/src/official_nmpc_benchmark.cpp
    )
    set(MINISOLVER_BENCH_INCLUDES
        targets/minisolver/generated
    )
    set(MINISOLVER_BENCH_DEFINES)

    if(MINISOLVER_CASE STREQUAL "pendulum_on_cart")
        list(APPEND MINISOLVER_BENCH_DEFINES MINISOLVER_CASE_PENDULUM_ON_CART=1)
    elseif(MINISOLVER_CASE STREQUAL "race_cars")
        list(APPEND MINISOLVER_BENCH_DEFINES MINISOLVER_CASE_RACE_CARS=1)
        set(RACE_GENERATED_DIR "${ACADOS_OUTPUT_ROOT}/race_cars/generated")
        if(NOT EXISTS "${RACE_GENERATED_DIR}/Spatialbicycle_model_model/Spatialbicycle_model_expl_vde_forw.c")
            message(FATAL_ERROR "Missing race_cars generated model assets under ${RACE_GENERATED_DIR}")
        endif()
        list(APPEND MINISOLVER_BENCH_SOURCES
            "${RACE_GENERATED_DIR}/Spatialbicycle_model_model/Spatialbicycle_model_expl_vde_forw.c"
            "${RACE_GENERATED_DIR}/Spatialbicycle_model_constraints/Spatialbicycle_model_constr_h_fun_jac_uxt_zt.c"
        )
        list(APPEND MINISOLVER_BENCH_INCLUDES
            "${RACE_GENERATED_DIR}/Spatialbicycle_model_model"
            "${RACE_GENERATED_DIR}/Spatialbicycle_model_constraints"
        )
    elseif(MINISOLVER_CASE STREQUAL "quadrotor_nav")
        list(APPEND MINISOLVER_BENCH_DEFINES MINISOLVER_CASE_QUADROTOR_NAV=1)
        set(QUAD_GENERATED_DIR "${ACADOS_OUTPUT_ROOT}/quadrotor_nav/generated")
        if(NOT EXISTS "${QUAD_GENERATED_DIR}/drone_FrenSer_model/drone_FrenSer_expl_vde_forw.c")
            message(FATAL_ERROR "Missing quadrotor_nav generated model assets under ${QUAD_GENERATED_DIR}")
        endif()
        list(APPEND MINISOLVER_BENCH_SOURCES
            "${QUAD_GENERATED_DIR}/drone_FrenSer_model/drone_FrenSer_expl_vde_forw.c"
            "${QUAD_GENERATED_DIR}/drone_FrenSer_constraints/drone_FrenSer_constr_h_fun_jac_uxt_zt.c"
        )
        list(APPEND MINISOLVER_BENCH_INCLUDES
            "${QUAD_GENERATED_DIR}/drone_FrenSer_model"
            "${QUAD_GENERATED_DIR}/drone_FrenSer_constraints"
        )
    elseif(MINISOLVER_CASE STREQUAL "chain_mass")
        list(APPEND MINISOLVER_BENCH_DEFINES MINISOLVER_CASE_CHAIN_MASS=1)
    else()
        message(FATAL_ERROR "Unsupported MINISOLVER_CASE: ${MINISOLVER_CASE}")
    endif()

    add_executable(minisolver_official_case_benchmark ${MINISOLVER_BENCH_SOURCES})
    target_compile_definitions(minisolver_official_case_benchmark PRIVATE ${MINISOLVER_BENCH_DEFINES})
    target_include_directories(minisolver_official_case_benchmark PRIVATE ${MINISOLVER_BENCH_INCLUDES})
    target_link_libraries(minisolver_official_case_benchmark PRIVATE minisolver_lib)

    add_executable(minisolver_asset_benchmark
        targets/minisolver/src/asset_benchmark.cpp
    )
    target_include_directories(minisolver_asset_benchmark PRIVATE "${NMPC_BENCH_COMMON_INCLUDE_DIR}")
    target_link_libraries(minisolver_asset_benchmark PRIVATE minisolver_lib)

    set(MINISOLVER_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/targets/minisolver/generated")
    set(KINEMATIC_BICYCLE_MODEL_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/targets/minisolver/models/kinematic_bicycle_asset/generate_model.py")
    set(KINEMATIC_BICYCLE_MODEL_HEADER "${MINISOLVER_GENERATED_DIR}/kinematicbicycletrackmodel.h")
    add_custom_command(
        OUTPUT "${KINEMATIC_BICYCLE_MODEL_HEADER}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${MINISOLVER_GENERATED_DIR}"
        COMMAND "${CMAKE_COMMAND}" -E env "MINISOLVER_SOURCE_DIR=${MINISOLVER_SOURCE_DIR}" "${Python3_EXECUTABLE}" "${KINEMATIC_BICYCLE_MODEL_SCRIPT}"
        DEPENDS
            "${KINEMATIC_BICYCLE_MODEL_SCRIPT}"
            "${CMAKE_CURRENT_SOURCE_DIR}/targets/minisolver/models/common.py"
            "${MINISOLVER_SOURCE_DIR}/python/minisolver/MiniModel.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        VERBATIM
    )
    add_custom_target(generate_minisolver_kinematic_bicycle_model DEPENDS "${KINEMATIC_BICYCLE_MODEL_HEADER}")

    set(DOUBLE_INTEGRATOR_3D_MODEL_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/targets/minisolver/models/double_integrator_3d_asset/generate_model.py")
    set(DOUBLE_INTEGRATOR_3D_MODEL_HEADER "${MINISOLVER_GENERATED_DIR}/doubleintegrator3dtrackingmodel.h")
    add_custom_command(
        OUTPUT "${DOUBLE_INTEGRATOR_3D_MODEL_HEADER}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${MINISOLVER_GENERATED_DIR}"
        COMMAND "${CMAKE_COMMAND}" -E env "MINISOLVER_SOURCE_DIR=${MINISOLVER_SOURCE_DIR}" "${Python3_EXECUTABLE}" "${DOUBLE_INTEGRATOR_3D_MODEL_SCRIPT}"
        DEPENDS
            "${DOUBLE_INTEGRATOR_3D_MODEL_SCRIPT}"
            "${CMAKE_CURRENT_SOURCE_DIR}/targets/minisolver/models/common.py"
            "${MINISOLVER_SOURCE_DIR}/python/minisolver/MiniModel.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        VERBATIM
    )
    add_custom_target(generate_minisolver_double_integrator_3d_model DEPENDS "${DOUBLE_INTEGRATOR_3D_MODEL_HEADER}")

    add_executable(minisolver_kinematic_bicycle_benchmark
        targets/minisolver/src/kinematic_bicycle_benchmark.cpp
        "${KINEMATIC_BICYCLE_MODEL_HEADER}"
    )
    add_dependencies(minisolver_kinematic_bicycle_benchmark generate_minisolver_kinematic_bicycle_model)
    target_include_directories(minisolver_kinematic_bicycle_benchmark PRIVATE "${NMPC_BENCH_COMMON_INCLUDE_DIR}" "${MINISOLVER_GENERATED_DIR}")
    target_link_libraries(minisolver_kinematic_bicycle_benchmark PRIVATE minisolver_lib)

    add_executable(minisolver_double_integrator_3d_benchmark
        targets/minisolver/src/double_integrator_3d_benchmark.cpp
        "${DOUBLE_INTEGRATOR_3D_MODEL_HEADER}"
    )
    add_dependencies(minisolver_double_integrator_3d_benchmark generate_minisolver_double_integrator_3d_model)
    target_include_directories(minisolver_double_integrator_3d_benchmark PRIVATE "${NMPC_BENCH_COMMON_INCLUDE_DIR}" "${MINISOLVER_GENERATED_DIR}")
    target_link_libraries(minisolver_double_integrator_3d_benchmark PRIVATE minisolver_lib)
endif()

if(CASADI_ROOT)
    if(NOT EXISTS "${CASADI_ROOT}/include/casadi/casadi.hpp")
        message(FATAL_ERROR "CASADI_ROOT does not point to a usable CasADi install: ${CASADI_ROOT}")
    endif()
    find_library(CASADI_LIBRARY NAMES casadi libcasadi.so PATHS "${CASADI_ROOT}" NO_DEFAULT_PATH)
    if(NOT CASADI_LIBRARY)
        find_library(CASADI_LIBRARY NAMES casadi PATHS "${CASADI_ROOT}" "${CASADI_ROOT}/lib" NO_DEFAULT_PATH)
    endif()
    if(NOT CASADI_LIBRARY)
        message(FATAL_ERROR "Could not find libcasadi under CASADI_ROOT=${CASADI_ROOT}")
    endif()

    add_executable(casadi_kinematic_bicycle_benchmark
        backends/casadi/src/kinematic_bicycle_benchmark.cpp
    )
    target_include_directories(casadi_kinematic_bicycle_benchmark PRIVATE "${NMPC_BENCH_COMMON_INCLUDE_DIR}" "${CASADI_ROOT}/include")
    target_compile_definitions(casadi_kinematic_bicycle_benchmark PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
    target_link_libraries(casadi_kinematic_bicycle_benchmark PRIVATE "${CASADI_LIBRARY}")
    set_target_properties(casadi_kinematic_bicycle_benchmark PROPERTIES BUILD_RPATH "${CASADI_ROOT}")

    add_executable(casadi_double_integrator_3d_benchmark
        backends/casadi/src/double_integrator_3d_benchmark.cpp
    )
    target_include_directories(casadi_double_integrator_3d_benchmark PRIVATE "${NMPC_BENCH_COMMON_INCLUDE_DIR}" "${CASADI_ROOT}/include")
    target_compile_definitions(casadi_double_integrator_3d_benchmark PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
    target_link_libraries(casadi_double_integrator_3d_benchmark PRIVATE "${CASADI_LIBRARY}")
    set_target_properties(casadi_double_integrator_3d_benchmark PROPERTIES BUILD_RPATH "${CASADI_ROOT}")
endif()
