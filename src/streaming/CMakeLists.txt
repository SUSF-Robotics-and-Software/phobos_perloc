
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(websocketpp REQUIRED)
pkg_check_modules(JSON REQUIRED json-glib-1.0)

add_library(Streaming
    Streaming.cpp
)

include_directories(
    ${JSON_INCLUDE_DIRS}
)

target_link_libraries(Streaming
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    ${JSON_LIBRARIES}
    nlohmann_json::nlohmann_json
    websocketpp::websocketpp
)