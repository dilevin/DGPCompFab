#include <igl/opengl/glfw/viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readMESH.h>

//assignment files
#include <GaussIncludes.h>
#include <FEMIncludes.h>
#include <ConstraintFixedPoint.h>
#include <projectDefines.h>

//Eigen
#include <Eigen/SparseCholesky>

using namespace Gauss;
using namespace Gauss::FEM;

//Define our linear finite element
template<typename DataType>
using LinearTetrahedron = ElementStatic<DataType, QuadratureLinearElasticityA4, ShapeFunctionTetLinearA4>;


//Setup the FE System
typedef PhysicalSystemFEM<double, LinearTetrahedron> LinearFEM;

typedef World<double, std::tuple<LinearFEM *>,
std::tuple<ForceSpringFEMParticle<double> *>,
std::tuple<ConstraintFixedPoint<double> *> > SimWorld;


//Global variables for UI
igl::opengl::glfw::Viewer viewer;

//This code from libigl boundary_facets.h
void tetsToTriangles(Eigen::MatrixXi &Fout, Eigen::MatrixXi T) {
    
    unsigned int simplex_size = 4;
    std::vector<std::vector<int> > allF(
                                   T.rows()*simplex_size,
                                        std::vector<int>(simplex_size-1));
    
    // Gather faces, loop over tets
    for(int i = 0; i< (int)T.rows();i++)
    {
        // get face in correct order
        allF[i*simplex_size+0][0] = T(i,2);
        allF[i*simplex_size+0][1] = T(i,3);
        allF[i*simplex_size+0][2] = T(i,1);
        // get face in correct order
        allF[i*simplex_size+1][0] = T(i,3);
        allF[i*simplex_size+1][1] = T(i,2);
        allF[i*simplex_size+1][2] = T(i,0);
        // get face in correct order
        allF[i*simplex_size+2][0] = T(i,1);
        allF[i*simplex_size+2][1] = T(i,3);
        allF[i*simplex_size+2][2] = T(i,0);
        // get face in correct order
        allF[i*simplex_size+3][0] = T(i,2);
        allF[i*simplex_size+3][1] = T(i,1);
        allF[i*simplex_size+3][2] = T(i,0);
    }
    
    Fout.resize(allF.size(), simplex_size-1);
    for(unsigned int ii=0; ii<allF.size(); ++ii) {
        Fout(ii,0) = allF[ii][0];
        Fout(ii,1) = allF[ii][1];
        Fout(ii,2) = allF[ii][2];
    }
}

int main(int argc, char **argv) {

    Eigen::MatrixXd V,C;
    Eigen::MatrixXi T,F;
    Eigen::VectorXd s;
    Eigen::VectorXi N;
    
    //Setup shape
    igl::readMESH(dataDir()+"/meshesTetWild/archbridge.mesh", V,T,F);
    
    tetsToTriangles(F, T);
    
    //Setup simulation code
    SimWorld world;
    LinearFEM *fem = new LinearFEM(V,T);
    world.addSystem(fem);
    world.finalize();
    
    //Assemble Stiffness Matrix and get forces
    AssemblerEigenSparseMatrix<double> K;
    AssemblerEigenVector<double> f;
    getStiffnessMatrix(K,world);
    getForceVector(f, world);
    
    //Project out fixed boundary
    Eigen::VectorXi fixedVertices = minVertices(fem, 1, 1e-3);
    
    //increase gravity
    Eigen::Vector3d g = {0.f, -100.f, 0.f};
    for(unsigned int ii=0; ii<fem->getImpl().getElements().size(); ++ii) {
        fem->getImpl().getElements()[ii]->setGravity(g);
    }
    
    Eigen::SparseMatrix<double> P = fixedPointProjectionMatrix(fixedVertices, *fem, world);
    Eigen::SparseMatrix<double> Kp = P*(*K)*P.transpose();
    Eigen::VectorXd fp = P*(*f);
    
    //Solve and project back to the full space
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> LDLT(Kp);
    mapStateEigen<0>(world) = P.transpose()*LDLT.solve(fp);
    
    //Display norm of stress
    C.resize(V.rows(),3);
    N.resize(V.rows());
    N.setZero();
    Eigen::Matrix3d S;
    s.resize(V.rows());
    
    for(unsigned int ii=0; ii< T.rows(); ii++) {
        
        for(unsigned int jj=0; jj<4; ++jj) {
            N[T(ii,jj)] += 1; //get normalization values for each vertex
        }
    }
    
    for(unsigned int ii=0; ii< T.rows(); ii++) {
        
        for(unsigned int jj=0; jj<4; ++jj) {
            fem->getImpl().getElement(ii)->getCauchyStress(S, Vec3d(0,0,0), world.getState());
            s(T(ii,jj)) += S.norm()/static_cast<double>(N(T(ii,jj)));
        }
    }
    
    igl::jet(s,0, 100000, C);
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();

}
