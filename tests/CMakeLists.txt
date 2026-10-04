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
  imu_kinematics_test.cpp
  normal_equations_test.cpp
  scan_to_map_test.cpp
  ndt_map_test.cpp)

# the code under test comes from the layer libraries (lo_infrastructure brings lo_application and lo_domain); core is
# not linked
target_link_libraries(lidar_odometry_tests PRIVATE lo_infrastructure)

target_include_directories(
  lidar_odometry_tests
  PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY}/doctest
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

# built without WITH_GUI: the code under test must not depend on the GUI
target_compile_definitions(lidar_odometry_tests PRIVATE LO_REPO_DIR="${REPOSITORY_DIRECTORY}")

if (MSVC)
  target_compile_options(lidar_odometry_tests PRIVATE /bigobj)
endif()

add_test(NAME lidar_odometry_tests COMMAND lidar_odometry_tests)
