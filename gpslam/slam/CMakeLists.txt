# Install headers
file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gpslam/slam)

# Build tests
gtsamAddTestsGlob(slam "tests/*.cpp" "" ${PROJECT_NAME})
