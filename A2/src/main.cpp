
//#include "util/shapes.hpp"
#include <igl/opengl/glfw/viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readDMAT.h>

//assignment files
#include <Optimization.h>
#include "Optimization.inl"
#include <csgOps.h>
#include <Shape.h>
#include <shapes.h>
#include <interopUtils.h>

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
    
    //Parameteric robot arm shape
    //Robot arm joint angles
    Eigen::VectorXf theta(2);
    theta<<0.0, 0.f;
    ShapeVar jointAngles("theta", theta);
    
    Shape myShape;
    myShape.registerVariable(jointAngles);
    
    //robot arm is made of two bars, rotated by two joint angles.
    myShape = rotate2d(
                       unionAB(box({-0.25f, -2.f, -.25f},{0.25f, 0.0f, .25f}),
                               move(rotate2d(box({-0.25f, -2.f, -.25f},{0.25f, 0.0f, .25f}), jointAngles(1)), {0.f, -2.0f, 0.f})),
                       jointAngles(0));
    
    
    //Target Point (bring robot arm in contact with this point)
    Eigen::Vector3f p(4.0,-0.1,0.0);
    
    //cost function
    auto objective = [&p, &myShape, &jointAngles](Eigen::VectorXd &x) {
        //set shape parameter values
        jointAngles[0] = x[0];
        jointAngles[1] = x[1];
        
        //compute cost function here and return it.
        return  0.0;
    };
    
    //gradient function
    auto gradient = [&p, &myShape, &jointAngles](Eigen::VectorXd &g, Eigen::VectorXd &x) {
        
        jointAngles[0] = x[0];
        jointAngles[1] = x[1];
        
        //compute the Jacobian (matrix of first derivatives of the cost function.
        //the function myShape.jacobian(g,p) will return the vector dImplicit/dAngles evaluated at point p
        
        //Set g
        //g =
        
    };
    
    
    //initial point
    Eigen::VectorXd x0(2);
    x0<<jointAngles[0],jointAngles[1];
    
    //Uncomment the lines below to run your various optimizers
    //std::cout<<"Final Cost: "<<gradientDescent(x0, objective, gradient, 1)<<"\n"; //Optimize using gradient descent with fixed step size. Try using step size 1, 0.5 and 0.1
    //std::cout<<"Final Cost: "<<gradientDescentWithBacktracking(x0, objective, gradient)<<"\n"; //Optimize using gradient descent with backtracking line search
    //std::cout<<"Final Cost: "<<newtonWithBacktracking(x0, objective, gradient)<<"\n"; //Optimize using newton's method with backtracking line search
    
    jointAngles[0] = x0[0];
    jointAngles[1] = x0[1];
    
    Kernel::Region<3> r({ -5, -5, -5 }, { 5, 5, 5 });

    fiveToIGL(V, F, *drawMesh(myShape, r, 0.2));
    
    viewer.data().set_mesh(V, F);
    viewer.launch();

}
