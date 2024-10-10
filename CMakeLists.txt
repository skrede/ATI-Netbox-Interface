cmake_minimum_required(VERSION 3.29)
project(Netbox)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
set(SIMPLE_SOCKET_BUILD_TESTS OFF)
FetchContent_Declare(
    SimpleSocket
    GIT_REPOSITORY https://github.com/markaren/SimpleSocket.git
    GIT_TAG e1ff5bba6486dd31c0605bbfef5f32a3baf44769
)
FetchContent_MakeAvailable(SimpleSocket)

add_subdirectory(netbox_demo)
add_subdirectory(netbox_interface)

add_dependencies(netbox_demo netbox_interface)