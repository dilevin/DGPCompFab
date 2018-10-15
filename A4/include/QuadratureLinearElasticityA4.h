#include <Energy.h>

template<typename DataType, typename Energy>
class QuadratureLinearElasticityA4 : public Energy {
    
public:
    
    using Energy::dphi;
    using Energy::m_qDofs;
    using Energy::m_qDotDofs;
    using Energy::m_E;
    using Energy::m_mu;
    
    template<typename QDOFList, typename QDotDOFList>
    inline QuadratureLinearElasticityA4(Eigen::MatrixXd &V, Eigen::MatrixXi &F, QDOFList &qDOFList, QDotDOFList &qDotDOFList) : Energy(V,F, qDOFList, qDotDOFList) {
        
        setParameters(m_E, m_mu);
        
    }
    
    inline QuadratureLinearElasticityA4() { }
    
    inline void setParameters(double youngsModulus, double poissonsRatio) {
        
        m_E = youngsModulus;
        m_mu = poissonsRatio;
        
        Eigen::Matrix<DataType, 6, 6> C;
        C.setZero();
        
        //Setup the C matrix from the lecture notes
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        Eigen::Matrix<DataType, 6,12> B;
        B.setZero();
        
        //Setup the B matrix from the lecture notes
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        m_K.setZero();
        
        //Setup the per-element stiffness matrix K and assign it to m_K
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
    }
    
    //integral rules for things that I want
    inline DataType getValue(const State<DataType> &state) {
        
        //Do nothing for now
        //returning the force which is really the negative gradient
        Eigen::Map<Eigen::VectorXd> v0 = mapDOFEigen(*m_qDofs[0], state);
        Eigen::Map<Eigen::VectorXd> v1 = mapDOFEigen(*m_qDofs[1], state);
        Eigen::Map<Eigen::VectorXd> v2 = mapDOFEigen(*m_qDofs[2], state);
        Eigen::Map<Eigen::VectorXd> v3 = mapDOFEigen(*m_qDofs[3], state);
        
        Eigen::Matrix<double, 12,1> u;
        u << v0, v1, v2, v3;
        
        double energy = 0.0;
        //u is a vector of displacements for this tetrahedral element. Use u and K to compute
        //the potential energy of this element.
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        return energy;
        
    }
    
    template<typename Vector>
    inline void getGradient(Vector &f, const State<DataType> &state) {
        
        //returning the force which is really the negative gradient
        Eigen::Map<Eigen::VectorXd> v0 = mapDOFEigen(*m_qDofs[0], state);
        Eigen::Map<Eigen::VectorXd> v1 = mapDOFEigen(*m_qDofs[1], state);
        Eigen::Map<Eigen::VectorXd> v2 = mapDOFEigen(*m_qDofs[2], state);
        Eigen::Map<Eigen::VectorXd> v3 = mapDOFEigen(*m_qDofs[3], state);
        
        Eigen::Matrix<double, 12,1> u;
        u << v0, v1, v2, v3;
        Eigen::Matrix<double, 12,1> f0;
        
        f0.setZero();
        
        //Use u and K to compute the internal forces of this energy. Rememebr the relationship
        //between potential energy and force.
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        assign(f, f0, m_qDofs);
        
    }
    
    template<typename Matrix>
    inline void getHessian(Matrix &H, const State<DataType> &state) {
        
        Eigen::Matrix<DataType, 12,12> stiffness;
        stiffness.setIdentity();
    
        //set H to the value of the stiffness martrix for this element. Remember the relationship
        //between the potential energy and the stiffness matrix.
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        assign(H, stiffness, m_qDofs, m_qDofs);
        
    }
    
protected:
    Eigen::Matrix<DataType, 12,12> m_K; //storage for the K matrix from class
private:
};
