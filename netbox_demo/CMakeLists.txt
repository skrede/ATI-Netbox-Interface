find_package(Eigen3 CONFIG REQUIRED)

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)

find_package(implot CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_executable(netbox_demo
        main.cpp
)

target_link_libraries(netbox_demo
    PUBLIC
    Eigen3::Eigen
    netbox_interface
    OpenGL::GL
    imgui::imgui
    implot::implot
    PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
)