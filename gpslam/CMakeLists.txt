# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set(gpslam_subdirs 
    gp 
    slam 
)
set(gpslam_srcs)

# files want to be excluded
set(excluded_sources "")


# Library sources
foreach(subdir ${gpslam_subdirs})
    file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
    list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

    file(GLOB subdir_test_files "${subdir}/tests/*")
  
    list(APPEND gpslam_srcs ${subdir_srcs})
    message(STATUS "Building Module: ${subdir}")

    # local and tests
    add_subdirectory(${subdir})
endforeach(subdir)


# build shared lib
add_library(${PROJECT_NAME} SHARED ${gpslam_srcs})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBS})

# Install library
install(TARGETS ${PROJECT_NAME} EXPORT gpslam-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)


