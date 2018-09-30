//
//  TimeStepperEulerImplicitLinear.h
//  Gauss
//
//  Created by David Levin on 2/10/17.
//
//

#ifndef TimeStepperEulerImplicitLinear_h
#define TimeStepperEulerImplicitLinear_h

#include <World.h>
#include <Assembler.h>
#include <TimeStepper.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <UtilitiesEigen.h>
#include <UtilitiesMATLAB.h>
#include <Eigen/SparseCholesky>
#include <SolverPardiso.h>

//TODO Solver Interface
namespace Gauss {
    
    //Given Initial state, step forward in time using linearly implicit Euler Integrator
    template<typename DataType, typename MatrixAssembler, typename VectorAssembler>
    class TimeStepperImplEulerImplicitLinear
    {
    public:
        
        TimeStepperImplEulerImplicitLinear() {
            std::cout<<"Using A3 Semi-Implicit Integrator \n";
        }
        
        TimeStepperImplEulerImplicitLinear(const TimeStepperImplEulerImplicitLinear &toCopy) {
            
        }
        
        ~TimeStepperImplEulerImplicitLinear() {
            
        }
        
        //Methods
        template<typename World>
        void step(World &world, double dt, double t);
        
    protected:
        
        MatrixAssembler m_massMatrix;
        MatrixAssembler m_stiffnessMatrix;
        VectorAssembler m_forceVector;
        
    private:
    };
}

//dt - the time step (sometimes reffered to as h)
template<typename DataType, typename MatrixAssembler, typename VectorAssembler>
template<typename World>
void TimeStepperImplEulerImplicitLinear<DataType, MatrixAssembler, VectorAssembler>::step(World &world, double dt, double t) {

    
    Eigen::SparseMatrix<DataType, Eigen::RowMajor> A;
    Eigen::VectorXd x0, b;
    
    MatrixAssembler &massMatrix = m_massMatrix;
    MatrixAssembler &stiffnessMatrix = m_stiffnessMatrix;
    
    //Get the mass matrix
    ASSEMBLEMATINIT(massMatrix, world.getNumQDotDOFs(), world.getNumQDotDOFs());
    ASSEMBLELIST(massMatrix, world.getSystemList(), getMassMatrix);
    ASSEMBLEEND(massMatrix);
    
    
    //Get the stiffness matrix
    ASSEMBLEMATINIT(stiffnessMatrix, world.getNumQDotDOFs(), world.getNumQDotDOFs());
    ASSEMBLELIST(stiffnessMatrix, world.getSystemList(), getStiffnessMatrix);
    ASSEMBLELIST(stiffnessMatrix, world.getForceList(), getStiffnessMatrix);
    ASSEMBLEEND(stiffnessMatrix);
    
    VectorAssembler &forceVector = m_forceVector;
    
    ASSEMBLEVECINIT(forceVector, world.getNumQDotDOFs()+world.getNumConstraints());
    ASSEMBLELIST(forceVector, world.getForceList(), getForce);
    ASSEMBLELIST(forceVector, world.getSystemList(), getForce);
    ASSEMBLELISTOFFSET(forceVector, world.getConstraintList(), getDbDt, world.getNumQDotDOFs(), 0);
    ASSEMBLEEND(forceVector);

    //Important Variables:
    
    //q - the displacement of each vertex in the simulated mesh, stored as a stacked vector [u1x, u1y, u1z ... unx, uny, unz]'. Here (u1x,u1y,u1z) is the 3D displacement of the first point and (unx, uny,unz) is the 3D displacement of the n^{th} point.
    Eigen::Map<Eigen::VectorXd> q = mapStateEigen<0>(world);
    
    //qDot - the velocity of each vertex in the simulated mesh, stored as a stacked vector [v1x, v1y, v1z ... vnx, vny, vnz]'. Here (v1x,v1y,v1z) is the 3D velocity of the first point and (unx, uny,unz) is the 3D velocity of the n^{th} point.
    Eigen::Map<Eigen::VectorXd> qDot = mapStateEigen<1>(world);
    
    //****** Using Mass, Stiffness and Force Matrix *********//
    //GAUSS will build the mass, stiffness and force vectors for the mesh being simulated using assemblers. In order
    //to use these in calculations you can convert them to Eigen vectors (linear algebra objects) using the * operator.
    //e.g If you want to add the mass matrix to the stiffness matrix you can write (*massMatrix)+(*stiffnessMatrix).
    //    If you want to multiply the mass matrix by the stiffness matrix you can write (*massMatrix)*(*forceVector).
    
    //---- YOUR CODE HERE ----//
    //Replace the next two lines with formulas to compute the lhs and rhs such that solving the equation
    //Ax = b implements a semi-implicit integration step
    A = (*massMatrix);
    b = Eigen::VectorXx<DataType>::Zero((*forceVector).rows(),1);
    //---- END YOUR CODE HERE ----//
    
    //Solve Linear System
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    solver.compute(A);
    
    
    if(solver.info()!=Eigen::Success) {
        // decomposition failed
        assert(1 == 0);
        std::cout<<"Decomposition Failed \n";
        exit(1);
    }
    
    if(solver.info()!=Eigen::Success) {
        // solving failed
        assert(1 == 0);
        std::cout<<"Solve Failed \n";
        exit(1);
    }
    //End Solve Linear System
    
    //Update State
    x0 = solver.solve(b);
    
    qDot = x0.head(world.getNumQDotDOFs());
    
    updateState(world, world.getState(), dt);
    //End Update State
}

template<typename DataType, typename MatrixAssembler, typename VectorAssembler>
using TimeStepperEulerImplicitLinear = TimeStepper<DataType, TimeStepperImplEulerImplicitLinear<DataType, MatrixAssembler, VectorAssembler> >;





#endif /* TimeStepperEulerImplicitLinear_h */
