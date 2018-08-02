
//#include "util/shapes.hpp"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readDMAT.h>

//assignment files
#include <csgOps.h>

//Global variables for UI
igl::opengl::glfw::Viewer viewer;

Eigen::MatrixXd V;
Eigen::MatrixXi F;

//interpreter
std::string currentFilename;
std::string currentObject;

//convert preprocessor define into a string
#define STRINGIFY(s) #s

#define DataDir(s) STRINGIFY(s)

int main(int argc, char **argv) {
    
    //custom menu to just-in-time compile libfive script
    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    currentObject = "";
    menu.callback_draw_custom_window = [&]()
    {
        // Define next window position + size
        ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
        ImGui::Begin(
                     "New Window", nullptr,
                     ImGuiWindowFlags_NoSavedSettings
                     );
        
        // Expose the same variable directly ...
        // Add a button
        if (ImGui::Button("Load Script", ImVec2(-1,0)))
        {
            currentFilename = igl::file_dialog_open();
            
        }
        
        if (ImGui::Button("Recompile", ImVec2(-1,0)))
        {
            if(currentFilename.length() != 0) {
                currentObject = "";
            
                //compile using command line interpreter and load mesh
                system((std::string("../interpreter/bin/Interpreter ") + currentFilename).c_str());
                igl::readDMAT("./outputV.dmat", V);
                igl::readDMAT("./outputF.dmat", F);
            }
            
            viewer.data().clear();
            viewer.data().set_mesh(V, F);
        }
    };

    viewer.launch();
}
