//Adapted from: https://github.com/ocornut/imgui/blob/master/examples/example_sdl2_opengl2/main.cpp
#include "sensor_interface/sensorcontroller.h"

#include <condition_variable>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include <SDL2/SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif
#include <implot.h>
#include <chrono>

#include "sensor_interface/sensorcontroller.h"

int main(int, char **)
{
    // Setup SDL
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }

    // Setup window
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window *window = SDL_CreateWindow("Force/torque sensor demo", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer bindings
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init();


    // Main loop
    bool render = true;
    bool pause_f = false;
    bool pause_t = false;
    std::vector<double> time_data;
    std::vector<double> bfx, bfy, bfz;
    std::vector<double> btx, bty, btz;
    std::vector<double> fx, fy, fz;
    std::vector<double> tx, ty, tz;

    std::mutex mutex;
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Vector3d fb = Eigen::Vector3d::Zero();
    Eigen::Vector3d tb = Eigen::Vector3d::Zero();

    auto controller = std::make_unique<estimation::sensor_interface::SensorController>("192.168.1.8", 49152u);
    controller->addSensorReadingReceivedListener(
        [&](const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
        {
            std::lock_guard<std::mutex> l(mutex);
            f = force;
            t = torque;
        }
    );
    auto push_value = [](std::vector<double> &vec, double v)
    {
        vec.insert(vec.begin(), v);
        if(vec.size() == 1000u)
            vec.pop_back();
    };

    auto update_plot = [&]()
    {
        {
            std::lock_guard<std::mutex> l(mutex);
            if(time_data.size() != 1000u)
                time_data.push_back((time_data.size() + 1u) / 100.0);
            push_value(bfx, f.x());
            push_value(bfy, f.y());
            push_value(bfz, f.z());
            push_value(btx, t.x());
            push_value(bty, t.y());
            push_value(btz, t.z());
        }
        if(!pause_f)
        {
            fx = bfx;
            fy = bfy;
            fz = bfz;
        }
        if(!pause_t)
        {
            tx = btx;
            ty = bty;
            tz = btz;
        }
        for(auto i = 0; i < fx.size(); i++)
        {
            if(!pause_f)
            {
                fx[i] += fb.x();
                fy[i] += fb.y();
                fz[i] += fb.z();
            }
            if(!pause_t)
            {
                tx[i] += tb.x();
                ty[i] += tb.y();
                tz[i] += tb.z();
            }
        }
    };
    while(render)
    {
        update_plot();
        SDL_Event event;
        while(SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if(event.type == SDL_QUIT)
                render = false;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        {
            ImGui::Begin("Forces");
            if(ImPlot::BeginPlot("Force X"))
            {
                ImPlot::PlotLine("Fx", &time_data[0], &fx[0], fx.size());
                ImPlot::EndPlot();
            }
            if(ImPlot::BeginPlot("Force Y"))
            {
                ImPlot::PlotLine("Fy", &time_data[0], &fy[0], fy.size());
                ImPlot::EndPlot();
            }
            if(ImPlot::BeginPlot("Force Z"))
            {
                ImPlot::PlotLine("Fz", &time_data[0], &fz[0], fz.size());
                ImPlot::EndPlot();
            }
            ImGui::End();
        }

        {
            ImGui::Begin("Torques");
            if(ImPlot::BeginPlot("Torque X"))
            {
                ImPlot::PlotLine("Tx", &time_data[0], &tx[0], tx.size());
                ImPlot::EndPlot();
            }
            if(ImPlot::BeginPlot("Torque Y"))
            {
                ImPlot::PlotLine("Ty", &time_data[0], &ty[0], ty.size());
                ImPlot::EndPlot();
            }
            if(ImPlot::BeginPlot("Torque Z"))
            {
                ImPlot::PlotLine("Tz", &time_data[0], &tz[0], tz.size());
                ImPlot::EndPlot();
            }
            ImGui::End();
        }

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            float min = -20.f;
            float max = 20.f;
            float bias[3] = {static_cast<float>(fb.x()), static_cast<float>(fb.y()), static_cast<float>(fb.z())};

            ImGui::Begin("Force"); // Create a window called "Hello, world!" and append into it.
            ImGui::SliderFloat("Bias Fx", &bias[0], min, max); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Bias Fy", &bias[1], min, max); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Bias Fz", &bias[2], min, max); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::InputFloat3("Bias F", (float*)&bias); // Edit 3 floats

            if(ImGui::Button("Reset")) // Buttons return true when clicked (most widgets return true when edited/activated)
            {
                bias[0] = 0.0;
                bias[1] = 0.0;
                bias[2] = 0.0;
            }
            ImGui::SameLine();
            if(ImGui::Button("Pause"))
            {
                pause_f = !pause_f;
            }
            ImGui::SameLine();
            ImGui::Text("Paused: %s", pause_f ? "true" : "false");
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
            fb = Eigen::Vector3d(bias[0], bias[1], bias[2]);
        }

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            float min = -2.f;
            float max = 2.f;
            float bias[3] = {static_cast<float>(tb.x()), static_cast<float>(tb.y()), static_cast<float>(tb.z())};

            ImGui::Begin("Torque"); // Create a window called "Hello, world!" and append into it.

            ImGui::SliderFloat("Bias Tx", &bias[0], min, max); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Bias Ty", &bias[1], min, max); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Bias Tz", &bias[2], min, max); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::InputFloat3("Bias T", (float*)&bias); // Edit 3 floats

            if(ImGui::Button("Reset")) // Buttons return true when clicked (most widgets return true when edited/activated)
            {
                bias[0] = 0.0;
                bias[1] = 0.0;
                bias[2] = 0.0;
            }
            ImGui::SameLine();
            if(ImGui::Button("Pause"))
            {
                pause_t = !pause_t;
            }
            ImGui::SameLine();
            ImGui::Text("Paused: %s", pause_t ? "true" : "false");
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
            tb = Eigen::Vector3d(bias[0], bias[1], bias[2]);
        }

        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClear(GL_COLOR_BUFFER_BIT);
        //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
