# Unit tests (doctest). They must run without recorded data, GUI or display.
add_executable(lidar_odometry_tests
  main.cpp
  smoke_test.cpp
  ${REPOSITORY_DIRECTORY}/app/toml_io.cpp)

target_include_directories(
  lidar_odometry_tests
  PRIVATE ${EXTERNAL_LIBRARIES_DIRECTORY}/doctest
          ${REPOSITORY_DIRECTORY}/app
          ${REPOSITORY_DIRECTORY}/core/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}
          ${EIGEN3_INCLUDE_DIR}
          ${EXTERNAL_LIBRARIES_DIRECTORY}/json/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${EXTERNAL_LIBRARIES_DIRECTORY}/observation_equations/codes
          ${EXTERNAL_LIBRARIES_DIRECTORY}/Fusion/Fusion)

if (MSVC)
  target_compile_options(lidar_odometry_tests PRIVATE /bigobj)
endif()

add_test(NAME lidar_odometry_tests COMMAND lidar_odometry_tests)
