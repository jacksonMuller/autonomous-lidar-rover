include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.5.3
)

FetchContent_MakeAvailable(Catch2)

add_executable(lidar_rover_tests
  test_scan_utils.cpp
  test_navigator.cpp
  scan_injector.cpp
  test_occupancy_grid.cpp
  test_lidar_mapper.cpp

)

target_include_directories(lidar_rover_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(lidar_rover_tests PRIVATE
  utils
  nav
  mapping
  Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(lidar_rover_tests)