find_package(Eigen3 CONFIG REQUIRED)

set(PUBLIC_HEADERS
    include/sensor_interface/netboxrdtclient.h
    include/sensor_interface/sensorcontroller.h
)

add_library(netbox_interface
    src/sensorcontroller.cpp
    src/netboxrdtclient.cpp

    ${PUBLIC_HEADERS}
)

target_link_libraries(netbox_interface
    PUBLIC
    -latomic
    simple_socket
    Eigen3::Eigen
)

target_include_directories(netbox_interface
    PUBLIC
    include/
    PRIVATE
)
