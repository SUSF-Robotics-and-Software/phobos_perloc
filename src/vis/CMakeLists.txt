add_library(Vis
    Vis.cpp
)

target_link_libraries(Vis
    ${OpenCV_LIBS}
    Filtering
)
