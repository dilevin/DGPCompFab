
//#include "util/shapes.hpp"
#include <igl/opengl/glfw/viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

//cling
#include <cling/Interpreter/Interpreter.h>
#include <cling/Interpreter/Value.h>
#include <cling/Utils/Casting.h>
#include <iostream>
#include <string>
#include <sstream>

//assignment files
#include <csgOps.h>

//Global variables for UI
igl::opengl::glfw::Viewer viewer;

Eigen::MatrixXd V;
Eigen::MatrixXi F;

//interpreter
cling::Interpreter *interp;
std::string currentFilename;
std::string currentObject;

//convert preprocessor define into a string
#define STRINGIFY(s) #s

#define DataDir(s) STRINGIFY(s)

//convenience functions for setting up libraries and include dirs
std::string eigenIncludeDir() {
    return std::string(DataDir(EIGEN_INCLUDE_DIR));
}

std::string a1IncludeDir() {
    return std::string(DataDir(A1_INCLUDE_DIR));
}

std::string libFiveIncludeDir() {
    return std::string(DataDir(LIBFIVE_INCLUDE_DIR));
}

std::string libFiveUtilsIncludeDir() {
    return std::string(DataDir(LIBFIVE_UTILS_INCLUDE_DIR));
}

std::string utilsIncludeDir() {
    return std::string(DataDir(UTILS_INCLUDE_DIR));
}

std::string libFiveDylib() {
    return std::string(DataDir(LIBFIVE_DYLIB));
}

std::string utilsDylib() {
    return std::string(DataDir(UTILS_DYLIB));
}

//setup headers for use with interpreter
void setupIncludePaths(cling::Interpreter  &interp) {
    
    //eigen
    interp.AddIncludePath(eigenIncludeDir());
    interp.AddIncludePath(a1IncludeDir());
    interp.AddIncludePath(libFiveIncludeDir());
    interp.AddIncludePath(libFiveUtilsIncludeDir());
    interp.AddIncludePath(utilsIncludeDir());

}

//setup external libraries
void setupExternalLibs(cling::Interpreter &interp) {
 
    //libfive
    interp.loadLibrary(libFiveDylib());
    interp.loadLibrary(utilsDylib());
    
}

std::string retunPointerCode(Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
    
    std::ostringstream sstr;
    
    sstr << "Eigen::MatrixXd& V = *(Eigen::MatrixXd *)" << std::hex << std::showbase << (size_t)&V << ';';
    sstr << "Eigen::MatrixXi& F = *(Eigen::MatrixXi *)" << std::hex << std::showbase << (size_t)&F << ';';
    return sstr.str();
}

std::string returnHeaderCode() {
    std::ostringstream sstr;

    sstr << "#include <iostream>\n";
    sstr<<  "#include <interopUtils.h>\n";
    sstr<<  "#include <shapes.h>\n";
    sstr<<  "#include <csgOps.h>\n";
    sstr<<  "#include <Eigen/Dense>\n";
    sstr<<  "#include <libfive/render/brep/mesh.hpp>\n";
    sstr<<  "#include <libfive/render/brep/xtree_pool.hpp>\n";
    sstr<<  "#include <libfive/render/brep/region.hpp>\n";
    sstr<<  "#include <libfive/render/brep/region.hpp>\n";
    sstr << "using namespace std;";
    sstr << "using namespace Kernel;";
    
    return sstr.str();
}

void recompileScript(cling::Interpreter &interp, std::string &object) {

    std::ostringstream sstr;
    sstr<< returnHeaderCode();
    sstr << retunPointerCode(V,F);
    
    sstr<<object;
    
    //PASS BACK TO IGL
    sstr<< " fiveToIGL(V,F, *mesh); ";
    interp.process(sstr.str(), nullptr, nullptr);
}

int main(int argc, char **argv) {
    
    interp = new cling::Interpreter(argc, argv, LLVMDIR);
    setupIncludePaths(*interp);
    setupExternalLibs(*interp);
    
    // Update the value of res by passing it to the interpreter.
    std::ostringstream sstr;
    
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
            if(currentFilename.length() == 0) {
                currentObject = "";
            } else {
                
                //delete interp;
                interp = new cling::Interpreter(argc, argv, LLVMDIR);
                setupIncludePaths(*interp);
                setupExternalLibs(*interp);
                
                currentObject = "";
                std::ifstream t(currentFilename);
                
                t.seekg(0, std::ios::end);
                currentObject.reserve(t.tellg());
                t.seekg(0, std::ios::beg);
                
                currentObject.assign((std::istreambuf_iterator<char>(t)),
                                     std::istreambuf_iterator<char>());
                std::cout<<currentObject<<"DONE\n";
            }
            
            recompileScript(*interp, currentObject);
            viewer.data().clear();
            viewer.data().set_mesh(V, F);
        }
    };

    viewer.launch();
}
