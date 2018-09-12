#include <Eigen/Dense>

//cling
#include <cling/Interpreter/Interpreter.h>
#include <cling/Interpreter/Value.h>
#include <cling/Utils/Casting.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
//assignment files
#include <csgOps.h>

//IGL
#include <igl/writeDMAT.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

//interpreter
cling::Interpreter *interp;
std::string currentFilename;
std::string currentObject;
std::vector<std::string> additionalIncludes;

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
    sstr<<  "#include <Shape.h>\n";
    sstr<<  "#include <csgOps.h>\n";
    sstr<<  "#include <Eigen/Dense>\n";
    sstr<<  "#include <libfive/render/brep/mesh.hpp>\n";
    sstr<<  "#include <libfive/render/brep/xtree_pool.hpp>\n";
    sstr<<  "#include <libfive/render/brep/region.hpp>\n";
    sstr<<  "#include <libfive/render/brep/region.hpp>\n";
    
    for(auto &fileStr : additionalIncludes) {
        std::cout<<"Add include file: "<<fileStr<<"\n";
        sstr<<"#include \""<<fileStr<<"\""<<"\n";
    }
    
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
    
    //rest of the args are user defined include files
    for(unsigned int iarg = 2; iarg < argc; ++iarg) {
        additionalIncludes.push_back(std::string(argv[iarg]));
    }
    
    
    interp = new cling::Interpreter(argc, argv, LLVMDIR);
    setupIncludePaths(*interp);
    setupExternalLibs(*interp);
    
    currentObject = "";
    std::string scriptFile(argv[1]);
    
    //first argument is the filename
    std::ifstream t(scriptFile);
    
    t.seekg(0, std::ios::end);
    currentObject.reserve(t.tellg());
    t.seekg(0, std::ios::beg);
    
    currentObject.assign((std::istreambuf_iterator<char>(t)),
                         std::istreambuf_iterator<char>());
    
    std::cout<<currentObject<<"\n";
    recompileScript(*interp, currentObject);
    std::cout<<"Compiled \n";
    //write out to file
    igl::writeDMAT("./outputV.dmat", V, false);
    igl::writeDMAT("./outputF.dmat", F, false);
}
