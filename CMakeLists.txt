cmake_minimum_required(VERSION 4.0.0)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# the application and executable name (dMapper.exe)
set(APP_NAME dMapper)
project(${APP_NAME})

add_definitions(-DHDMAPPING_VERSION_MAJOR=0)
add_definitions(-DHDMAPPING_VERSION_MINOR=95)
add_definitions(-DHDMAPPING_VERSION_PATCH=0)

include_directories(shared/include)

set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_CXX_STANDARD 20)

# CPU Architecture optimization options
set(HD_CPU_OPTIMIZATION "AUTO" CACHE STRING "CPU optimization target (AUTO, INTEL, AMD, ARM, GENERIC)")
set_property(CACHE HD_CPU_OPTIMIZATION PROPERTY STRINGS AUTO INTEL AMD ARM GENERIC)

if (MSVC)
    add_definitions(-D_HAS_STD_BYTE=0)

    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /GL")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2 /GL")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG")

    if(HD_CPU_OPTIMIZATION STREQUAL "AMD")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
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
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
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
include(cmake/imguizmo.cmake)
include(cmake/implot.cmake)

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

# GLEW is only used for its headers by lidar_odometry_gui.cpp (no glewInit()/GLEW
# symbols are actually called or linked); kept for the include path only, matching
# the original app's behavior.
add_subdirectory(${EXTERNAL_LIBRARIES_DIRECTORY}/glew-cmake)
set(GLEW_INCLUDE_DIR ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-cmake/include)

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

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/app/default_settings.toml"
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

# This must match core/CMakeLists.txt's CORE_SOURCE_FILES / CORE_SOURCE_FILES_GUI
# in the original hd-mapping repo exactly (not a glob of core/src/*.cpp). The
# original app links this as a static lib; the linker then only ever pulls a
# .obj out of the archive when something needs a symbol it uniquely provides.
# That's why hash_utils.cpp's get_index()/get_rgd_index() never collide with
# the app's own definitions of the same names, and why surface.cpp (which
# duplicates pose_graph_slam.cpp's global `rd`/`gen`) is simply never part of
# this list. Compiling every core/src/*.cpp flat into one exe (as core_no_gui-
# style single-TU builds do) breaks both of those and fails to link.
set(CORE_SOURCE_FILES
    core/src/color_las_loader.cpp
    core/src/gnss.cpp
    core/src/icp.cpp
    core/src/ndt.cpp
    core/src/nns.cpp
    core/src/nmea.cpp
    core/src/optimization_point_to_point_source_to_target.cpp
    core/src/optimize_distance_point_to_plane_source_to_target.cpp
    core/src/optimize_plane_to_plane_source_to_target.cpp
    core/src/optimize_point_to_plane_source_to_target.cpp
    core/src/optimize_point_to_projection_onto_plane_source_to_target.cpp
    core/src/pcl_wrapper.cpp
    core/src/plycpp.cpp
    core/src/point_cloud.cpp
    core/src/point_clouds.cpp
    core/src/pose_graph_slam.cpp
    core/src/pose_graph_loop_closure.cpp
    core/src/session.cpp
    core/src/wgs84_do_puwg92.cc
    core/src/pair_wise_iterative_closest_point.cpp
    core/src/hash_utils.cpp
    core/src/ground_control_points.cpp
    core/src/control_points.cpp
    core/src/registration_plane_feature.cpp
)

set(CORE_SOURCE_FILES_GUI
    core/src/pfd_wrapper.cpp
    core/src/manual_pose_graph_loop_closure.cpp
    core/src/observation_picking.cpp
)

add_library(core STATIC ${CORE_SOURCE_FILES} ${CORE_SOURCE_FILES_GUI})
target_compile_definitions(core PRIVATE WITH_GUI=1)
target_link_libraries(core PRIVATE ${PLATFORM_LASZIP_LIB} ${PLATFORM_MISCELLANEOUS_LIBS})
target_include_directories(core PRIVATE
    core/include
    ${EIGEN3_INCLUDE_DIR}
    ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
    ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
    ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
    ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
    ${LASZIP_INCLUDE_DIR}/LASzip/include
    ${FREEGLUT_INCLUDE_DIR}
    ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
    ${EXTERNAL_LIBRARIES_DIRECTORY}/manif/include
    ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs)

set(SOURCES
  ${REPOSITORY_DIRECTORY}/core/src/utils.cpp
  app/lidar_odometry_gui.cpp
  app/lidar_odometry.cpp
  app/lidar_odometry.h
  app/lidar_odometry_utils.h
  app/ndt_map.h
  app/ndt_map.cpp
  app/odometry_config.h
  app/odometry_state.h
  app/run_odometry.cpp
  app/run_odometry.h
  app/loop_closure.cpp
  app/loop_closure.h
  app/odometry_pipeline.cpp
  app/odometry_pipeline.h
  app/run_reporting.h
  app/console_reporting.cpp
  app/console_reporting.h
  app/input_ports.h
  app/recording_files.cpp
  app/recording_files.h
  app/disk_point_store.cpp
  app/disk_point_store.h
  app/folder_preview_sink.cpp
  app/folder_preview_sink.h
  app/recording_folder_source.cpp
  app/recording_folder_source.h
  app/folder_inputs.cpp
  app/folder_inputs.h
  app/output_ports.h
  app/window.h
  app/window_steps.cpp
  app/window_steps.h
  app/angle_units.h
  app/normal_equations.cpp
  app/normal_equations.h
  app/imu_kinematics.cpp
  app/imu_kinematics.h
  app/scan_to_map.cpp
  app/scan_to_map.h
  app/robust_lidar_odometry.cpp
  app/robust_lidar_odometry.h
  app/consistency.cpp
  app/consistency.h
  app/optimizer_options.cpp
  app/optimizer_options.h
  app/toml_config_repository.cpp
  app/toml_config_repository.h
  app/folder_result_sink.cpp
  app/folder_result_sink.h
  app/toml.hpp
  app/toml_io.h
  app/toml_io.cpp
)

if(WIN32)
    list(APPEND SOURCES app/resource.rc)
endif()

add_executable(${APP_NAME} ${SOURCES})

target_compile_definitions(${APP_NAME} PRIVATE -DWITH_GUI=1)

target_include_directories(
  ${APP_NAME}
  PRIVATE app
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui
          ${EXTERNAL_LIBRARIES_DIRECTORY}/imgui/backends
          ${EXTERNAL_LIBRARIES_DIRECTORY}/ImGuizmo
          ${EXTERNAL_LIBRARIES_DIRECTORY}/glew-cmake/include
          ${FREEGLUT_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/portable-file-dialogs
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion
          ${EXTERNAL_LIBRARIES_DIRECTORY}/manif/include)

target_link_libraries(
          ${APP_NAME}
          PRIVATE core
          Fusion
          ${FREEGLUT_LIBRARY}
          ${OPENGL_gl_LIBRARY}
          OpenGL::GLU
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          imgui
          imguizmo
          implot)

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
            "${CMAKE_CURRENT_SOURCE_DIR}/app/default_settings.toml"
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
