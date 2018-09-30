#include <igl/opengl/glfw/viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readMESH.h>
#include <igl/edges.h>
//assignment files
#include <GaussIncludes.h>
#include <ForceSpringA3.h>
#include <ForceParticleGravity.h>
#include <ConstraintFixedPoint.h>
#include <PhysicalSystemParticles.h>
#include <TimeStepperEulerImplicitLinearA3.h>

//Eigen
#include <Eigen/SparseCholesky>

using namespace Gauss;
using namespace Gauss::ParticleSystem;

typedef World<double, std::tuple<PhysicalSystemParticleSingle<double> *>,
std::tuple<ForceSpringParticles<double> *, ForceParticlesGravity<double> *>,
std::tuple<ConstraintFixedPoint<double> *> > SimWorld;

typedef TimeStepperEulerImplicitLinear<double, AssemblerEigenSparseMatrix<double>, AssemblerEigenVector<double>  > MyTimeStepper;

SimWorld world;
MyTimeStepper stepper(0.005);
Eigen::MatrixXd V, Vdef;
Eigen::MatrixXi F;

//Global variables for UI
igl::opengl::glfw::Viewer viewer;

//libigl viewer callback
bool postDrawCallback(igl::opengl::glfw::Viewer & viewer) {
    
    stepper.step(world);
    
    //simple collision detection and resolution with floor
    //check if particle is below the floor, if so, clamp y position to the floor
    double floorY = -5;
    for(unsigned int ii=0; ii<V.rows(); ++ii) {
        if((V(ii,1) + mapStateEigen<0>(world)[3*ii+1]) < floorY) {
            mapStateEigen<0>(world)[3*ii+1] = floorY - V(ii,1);
            mapStateEigen<1>(world)[3*ii+1] = 0.0;
        }
    }
    
    Eigen::VectorXd u = mapStateEigen<0>(world); //set displacements to zero
    
    for(unsigned int ii=0; ii<Vdef.rows(); ++ii) {
        Vdef(ii,0) = V(ii,0)+u[3*ii];
        Vdef(ii,1) = V(ii,1)+u[3*ii+1];
        Vdef(ii,2) = V(ii,2)+u[3*ii+2];
    }
    viewer.data().clear();
    viewer.data().set_mesh(Vdef, F);

    return false;
}

int main(int argc, char **argv) {

    Eigen::MatrixXi T, E;
    
    //Setup shape
    igl::readMESH("/Users/dilevin/Documents/Teaching/CompFabUofT/2018b/Instructor/ThirdParty/GAUSS/data/meshesTetWild/archbridge.mesh", V,T,F);
    
    //get edges of tetrahedra
    igl::edges(T,E);
    
    //Setup simulation code
    Eigen::Vector3d gravity = {0, -9.8, 0.0};
    //1. add every vertex of our mesh to the world as a paticle
    //2. add gravity to every particle
    for(unsigned int ii=0; ii<V.rows(); ++ii) {
        PhysicalSystemParticleSingle<double> *p = new PhysicalSystemParticleSingle<double>();
        world.addSystem(p);
        world.addForce(new ForceParticlesGravity<double>(&(p->getQ()), 1.0, gravity)); //particles are effected by gravity
        
        
    }
    
    //3. add spring forces to every edge in the triangle mesh
    double k = 2000; //Spring constant
    for(unsigned int ii=0; ii<E.rows(); ++ii) {
        world.addForce(new ForceSpringParticles<double>(
                                                        PosParticle<double>(&world.getSystemList().get<0>()[E(ii,0)]->getQ(), V.row(E(ii,0))),
                                                        PosParticle<double>(&world.getSystemList().get<0>()[E(ii,1)]->getQ(), V.row(E(ii,1))),
                                                        (V.row(E(ii,0)) - V.row(E(ii,1))).norm(), k));
    }
    
    
    world.finalize();
    
    Vdef = V;
    //set the displacement and the velocity of every spring to be 0 initially
    mapStateEigen<0>(world).setZero(); //set displacements to zero
    mapStateEigen<1>(world).setZero(); //set velocities to zero initally.
    
    viewer.callback_post_draw = &postDrawCallback;
    viewer.data().set_mesh(Vdef, F);
    viewer.launch();

}
