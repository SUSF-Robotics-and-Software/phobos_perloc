
# Must be here rather than in Streaming since we have to include gst in the
# streaming header, and that means this couldn't build
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
    ${GLIB_LIBRARY_DIRS}
    ${GSTREAMER_LIBRARY_DIRS}
)

# Libraries
add_subdirectory(data_source)
add_subdirectory(vis)
add_subdirectory(filtering)
add_subdirectory(streaming)

# Main executable
add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    ${OpenCV_LIBS}
    OakdDataSource
    Vis
    Streaming
)