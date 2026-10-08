cmake_minimum_required(VERSION 4.0.0)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# the application and executable name (dMapper.exe)
set(APP_NAME dMapper)
project(${APP_NAME})

# dMapper's version (src/domain/dmapper_version.h): in the result files' names and contents and the GUI title
add_definitions(-DDMAPPER_VERSION_MAJOR=0)
add_definitions(-DDMAPPER_VERSION_MINOR=1)
add_definitions(-DDMAPPER_VERSION_PATCH=0)

set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_CXX_STANDARD 20)

# CPU Architecture optimization options
set(HD_CPU_OPTIMIZATION "AUTO" CACHE STRING "CPU optimization target (AUTO, INTEL, AMD, ARM, GENERIC)")
set_property(CACHE HD_CPU_OPTIMIZATION PROPERTY STRINGS AUTO INTEL AMD ARM GENERIC)

if (MSVC)
    add_definitions(-D_HAS_STD_BYTE=0)

    # one floating-point mode (/fp:precise, MSVC's default) for every build: /fp:fast gained 0-2 % and made results depend on how the
    # compiler happened to order sums (see docs/PERFORMANCE.md)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /GL /fp:precise")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2 /GL /fp:precise")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG")

    if(HD_CPU_OPTIMIZATION STREQUAL "AMD")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /Oy")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Oi /Ot /Oy")
        message(STATUS "Enabling AMD-optimized build for MSVC")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
        message(STATUS "Enabling Intel-optimized build for MSVC")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "ARM")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
        message(STATUS "Enabling ARM-optimized build for MSVC")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "AUTO")
        cmake_host_system_information(RESULT CPU_VENDOR QUERY PROCESSOR_DESCRIPTION)
        cmake_host_system_information(RESULT CPU_ARCH QUERY PROCESSOR_DESCRIPTION)
        if(CPU_ARCH MATCHES "arm" OR CPU_ARCH MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
            message(STATUS "Auto-detected ARM processor - enabling ARM optimizations")
        elseif(CPU_VENDOR MATCHES "AMD")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /Oy")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Oi /Ot /Oy")
            message(STATUS "Auto-detected AMD processor - enabling AMD optimizations")
        else()
            message(STATUS "Auto-detected Intel/Generic processor - using conservative optimizations")
        endif()
    else()
        message(STATUS "Using generic optimizations for MSVC")
    endif()
else()
    if(HD_CPU_OPTIMIZATION STREQUAL "AMD")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
        message(STATUS "Enabling AMD-optimized build for GCC/Clang")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
        message(STATUS "Enabling Intel-optimized build for GCC/Clang")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "ARM")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
            message(STATUS "Enabling ARM64/AArch64 optimizations for GCC/Clang")
        else()
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
            message(STATUS "Enabling ARM32 optimizations for GCC/Clang")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "AUTO")
        cmake_host_system_information(RESULT CPU_VENDOR QUERY PROCESSOR_DESCRIPTION)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
                set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
                set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
                message(STATUS "Auto-detected ARM64 processor - enabling ARM optimizations")
            else()
                set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
                set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
                message(STATUS "Auto-detected ARM32 processor - enabling ARM optimizations")
            endif()
        elseif(CPU_VENDOR MATCHES "AMD")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
            message(STATUS "Auto-detected AMD processor - enabling AMD optimizations")
        else()
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
            message(STATUS "Auto-detected Intel/Generic processor - using Intel optimizations")
        endif()
    else()
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=x86-64 -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -march=x86-64 -DNDEBUG")
        message(STATUS "Using generic optimizations for GCC/Clang")
    endif()
endif()

find_package(OpenGL REQUIRED)

add_definitions(-D_USE_MATH_DEFINES)

get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(EXTERNAL_LIBRARIES_DIRECTORY ${REPOSITORY_DIRECTORY}/3rdparty)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(cmake/platform.cmake)
include(cmake/imgui.cmake)

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)
set(EIGEN3_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)
message(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")

set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut)
set(FREEGLUT_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/freeglut/include)
set(FREEGLUT_LIBRARY freeglut)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(freeglut PRIVATE -Wno-unused-const-variable)
endif()

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/LASzip)
set(LASZIP_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY})
set(LASZIP_LIBRARY laszip)

set(TBB_TEST OFF CACHE BOOL "" FORCE)
set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
set(TBB_STRICT OFF CACHE BOOL "" FORCE)
set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
set(TBB_CPF OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/oneTBB)

add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/settings/default_settings.toml"
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# ---- layers (docs/ARCHITECTURE.md): each library gets only the include directories its layer may use, so a file that
# includes something its layer must not see doesn't compile ----

# domain: entities and the optimizers. Eigen, manif and observation_equations only; no LASzip, json, GL, ImGui, Fusion
# or TBB directories
add_library(lo_domain STATIC
  src/domain/ndt_map.cpp
  src/domain/normal_equations.cpp
  src/domain/imu_kinematics.cpp
  src/domain/window_steps.cpp
  src/domain/optimizer_options.cpp
  src/domain/scan_to_map.cpp
  src/domain/robust_lidar_odometry.cpp)
target_include_directories(lo_domain PUBLIC
  src/domain
  ${EIGEN3_INCLUDE_DIR}
  ${EXTERNAL_LIBRARIES_DIRECTORY}/manif/include
  ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes)

# application: the use cases and the ports (input, output and service ports). Nothing third-party beyond the domain's:
# the AHRS (Fusion) and the parallel loops (TBB) are reached through OrientationEstimator and ParallelExecutor
add_library(lo_application STATIC
  src/application/odometry_pipeline.cpp
  src/application/lidar_odometry.cpp
  src/application/run_odometry.cpp
  src/application/loop_closure.cpp
  src/application/consistency.cpp)
target_include_directories(lo_application PUBLIC src/application)
target_link_libraries(lo_application PUBLIC lo_domain)

# infrastructure: the adapters (folders, LAZ, CSV, TOML, JSON, console, the Fusion AHRS, the TBB executor), with the
# include directories of those libraries only: no ImGui, GLUT or GL
add_library(lo_infrastructure STATIC
  src/infrastructure/recording_folder_source.cpp
  src/infrastructure/recording_files.cpp
  src/infrastructure/disk_point_store.cpp
  src/infrastructure/folder_preview_sink.cpp
  src/infrastructure/laz_export.cpp
  src/infrastructure/folder_inputs.cpp
  src/infrastructure/fusion_orientation_estimator.cpp
  src/infrastructure/run_services.cpp
  src/infrastructure/tbb_executor.cpp
  src/infrastructure/folder_result_sink.cpp
  src/infrastructure/toml_io.cpp
  src/infrastructure/toml_config_repository.cpp
  src/infrastructure/console_reporting.cpp)
target_include_directories(lo_infrastructure
  PUBLIC src/infrastructure
  PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/csv-parser
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)
target_link_libraries(lo_infrastructure PUBLIC lo_application PRIVATE ${PLATFORM_LASZIP_LIB} Fusion ${PLATFORM_MISCELLANEOUS_LIBS})

# presentation: the GUI's view-model (its state and actions). Only the infrastructure's include directories, no ImGui,
# GLUT, GL or file dialogs, so it builds and is tested without a display
add_library(lo_gui_view_model STATIC
  src/presentation/gui/gui_view_model.cpp)
target_include_directories(lo_gui_view_model PUBLIC src/presentation/gui)
target_link_libraries(lo_gui_view_model PUBLIC lo_infrastructure)

if (MSVC)
  target_compile_options(lo_domain PRIVATE /bigobj)
  target_compile_options(lo_application PRIVATE /bigobj)
  target_compile_options(lo_infrastructure PRIVATE /bigobj)
endif()

# presentation (CLI, GUI) and the composition root
set(SOURCES
  src/presentation/gui/viewer.cpp
  src/presentation/gui/viewer.h
  src/presentation/gui/file_dialogs.cpp
  src/presentation/gui/file_dialogs.h
  src/main.cpp
  src/presentation/format_time.h
  src/presentation/cli/cli.cpp
  src/presentation/cli/cli.h
  src/presentation/gui/lidar_odometry_gui.cpp
)

if(WIN32)
    list(APPEND SOURCES src/presentation/gui/resource.rc)
endif()

add_executable(${APP_NAME} ${SOURCES})


target_include_directories(
  ${APP_NAME}
  PRIVATE src/presentation
          src/presentation/cli
          src/presentation/gui
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${FREEGLUT_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion
          ${EXTERNAL_LIBRARIES_DIRECTORY}/manif/include)

target_link_libraries(
          ${APP_NAME}
          PRIVATE lo_gui_view_model
          lo_infrastructure
          Fusion
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          imgui)

if(WIN32)
  add_custom_command(
    TARGET ${APP_NAME}
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:${APP_NAME}>
      $<TARGET_FILE_DIR:${APP_NAME}>
    COMMAND_EXPAND_LISTS)

    add_custom_command(
    TARGET ${APP_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_SOURCE_DIR}/settings/default_settings.toml"
            $<TARGET_FILE_DIR:${APP_NAME}>
)
endif()

if (MSVC)
    target_compile_options(${APP_NAME} PRIVATE /bigobj)
endif()

install(TARGETS ${APP_NAME} DESTINATION bin)

# unit tests: build the lidar_odometry_tests target, run with ctest --test-dir out/build/<preset>
enable_testing()
add_subdirectory(tests)

# tools/cut_recording: cuts a time range of a recording into a new input folder (LASzip only)
add_executable(cut_recording tools/cut_recording/cut_recording.cpp)
target_include_directories(cut_recording PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include)
target_link_libraries(cut_recording PRIVATE ${PLATFORM_LASZIP_LIB})
if(WIN32)
  add_custom_command(
    TARGET cut_recording
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:cut_recording> $<TARGET_FILE_DIR:cut_recording>
    COMMAND_EXPAND_LISTS)
endif()
