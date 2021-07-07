add_library(Filtering
    Filtering.cpp
)

target_link_libraries(Filtering
    ${OpenCV_LIBS}
)
