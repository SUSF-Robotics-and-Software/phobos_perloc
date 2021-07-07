
find_package(depthai CONFIG REQUIRED)

add_library(OakdDataSource
    OakdDataSource.cpp
)

target_link_libraries(OakdDataSource
    ${OpenCV_LIBS}
    depthai::opencv
    Filtering
)
