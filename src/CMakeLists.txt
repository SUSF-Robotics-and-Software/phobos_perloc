
# Libraries
add_subdirectory(data_source)
add_subdirectory(vis)
add_subdirectory(filtering)

# Main executable
add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    ${OpenCV_LIBS}
    OakdDataSource
    Vis
    Filtering
)