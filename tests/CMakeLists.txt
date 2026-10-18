# Unit tests (doctest). They must run without recorded data, GUI or display.
add_executable(lidar_odometry_tests
  main.cpp
  smoke_test.cpp
  toml_golden_test.cpp
  run_reporting_test.cpp
  recording_files_test.cpp
  disk_point_store_test.cpp
  toml_config_repository_test.cpp
  optimizer_options_test.cpp
  window_steps_test.cpp
  holdout_test.cpp
  window_cut_test.cpp
  imu_kinematics_test.cpp
  normal_equations_test.cpp
  phase_timing_test.cpp
  scan_to_map_test.cpp
  ndt_map_test.cpp
  synthetic_recording.cpp
  synthetic_recording_test.cpp
  odometry_snapshot_test.cpp
  folder_result_sink_test.cpp
  loop_closure_test.cpp
  consistency_test.cpp
  orientation_test.cpp
  gui_view_model_test.cpp)

# the code under test comes from the layer libraries (lo_gui_view_model brings lo_infrastructure, lo_application and
# lo_domain)
target_link_libraries(lidar_odometry_tests PRIVATE lo_gui_view_model lo_infrastructure)

target_include_directories(
  lidar_odometry_tests
  PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY}/doctest
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes)

target_compile_definitions(lidar_odometry_tests PRIVATE LO_REPO_DIR="${REPOSITORY_DIRECTORY}")

if (MSVC)
  target_compile_options(lidar_odometry_tests PRIVATE /bigobj)
endif()

# the runtime DLLs (TBB) next to the test executable, as for dMapper
if(WIN32)
  add_custom_command(
    TARGET lidar_odometry_tests
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:lidar_odometry_tests> $<TARGET_FILE_DIR:lidar_odometry_tests>
    COMMAND_EXPAND_LISTS)
endif()

add_test(NAME lidar_odometry_tests COMMAND lidar_odometry_tests)

if (MSVC)
  target_compile_options(lidar_odometry_tests PRIVATE /we4715)
endif()
