//Linear Tetrahedral Shape Function
template<typename DataType>
class ShapeFunctionTetLinearA4 {
public:
    
    using MatrixJ = Eigen::Matrix<DataType, 3, 12>;
    using VectorQ = Eigen::Matrix<DataType, 12,1>;
    
    //convenience matrix type
    template<unsigned int Rows>
    using MatrixDOF = Eigen::Matrix<DataType, Rows, 12>;
    
    ShapeFunctionTetLinear() { }
    
    template<typename QDOFList, typename QDotDOFList>
    ShapeFunctionTetLinearA4(Eigen::MatrixXd &V, Eigen::MatrixXi &F, QDOFList &qDOFList, QDotDOFList &qDotDOFList) : m_T(3,3) {
        
        
        //for the time being assume things come as a stack (qdofs and qdotdofs)
        m_qDofs[0] = qDOFList[0];
        m_qDofs[1] = qDOFList[1];
        m_qDofs[2] = qDOFList[2];
        m_qDofs[3] = qDOFList[3];
        
        m_qDotDofs[0] = qDotDOFList[0];
        m_qDotDofs[1] = qDotDOFList[1];
        m_qDotDofs[2] = qDotDOFList[2];
        m_qDotDofs[3] = qDotDOFList[3];
    }
    
    //This function computes the value of the shape function associated with the vertex Vertex.
    template<unsigned int Vertex>
    inline double phi(double *x) {
        assert(Vertex >= 0);
        assert(Vertex < 4);
        
        DataType phiOut = 0.0;
        
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        return phiOut;
    }
    
    //The 3x1 gradient of phi with respect to X for phi<Vertex>
    template<unsigned int Vertex>
    inline std::array<DataType, 3> dphi(double *x) {
        
        temp[0] = 0.0;
        temp[1] = 0.0;
        temp[2] = 0.0;
        
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        return temp;
    }
    
    
    inline Eigen::Matrix<DataType, 3,3> F(double *x, const State<DataType> &state) {
        
        Eigen::Matrix<DataType,3,3> Ftemp;
        Ftemp.setIdentity();
        
        //---- YOUR CODE HERE ----//
        //---- END YOUR CODE HERE ----//
        
        return Ftemp;
        
    }
    
    //Shape function matrix at point x
    inline MatrixJ N(double *x) {
        
        MatrixJ output;
        
        //just a 3x12 matrix of shape functions
        //kind of assuming everything is initialized before we get here
        double phi0 = phi<0>(x);
        double phi1 = phi<1>(x);
        double phi2 = phi<2>(x);
        double phi3 = phi<3>(x);
        
        output.resize(3,12);
        output.setZero();
        output.block(0,0, 3,3) = phi0*Eigen::Matrix3d::Identity();
        output.block(0,3, 3,3) = phi1*Eigen::Matrix3d::Identity();
        output.block(0,6, 3,3) = phi2*Eigen::Matrix3d::Identity();
        output.block(0,9, 3,3) = phi3*Eigen::Matrix3d::Identity();
        
        return output;
        
    }
    
    
    //Jacobian: derivative with respect to degrees of freedom
    inline MatrixJ J(double *x, const State<DataType> &state) {
        
        return N(x);
        
    }
    
    
    //get DOFs
    inline const std::array<DOFBase<DataType,0> *, 4> & q() const {
        return m_qDofs;
    }
    
    inline const std::array<DOFBase<DataType,1> *, 4> & qDot() const {
        return m_qDotDofs;
    }
    
    inline VectorQ q(const State<DataType> &state) const {
        
        VectorQ tmp;
        
        tmp<<  mapDOFEigen(*m_qDofs[0], state),
        mapDOFEigen(*m_qDofs[1], state),
        mapDOFEigen(*m_qDofs[2], state),
        mapDOFEigen(*m_qDofs[3], state);
        return tmp;
    }
    
    inline VectorQ qDot(const State<DataType> &state) {
        
        VectorQ tmp;
        
        tmp<<  mapDOFEigen(*m_qDotDofs[0], state),
        mapDOFEigen(*m_qDotDofs[1], state),
        mapDOFEigen(*m_qDotDofs[2], state),
        mapDOFEigen(*m_qDotDofs[3], state);
        return tmp;
    }
    
    inline MatrixJ GradJ(unsigned int component, double *x, const State<DataType> &state) {
        
        MatrixJ tmp;
        
        tmp.setZero();
        tmp.col(component) = Eigen::Map3x<DataType>(dphi<0>(x).data());
        tmp.col(3+component) = Eigen::Map3x<DataType>(dphi<1>(x).data());
        tmp.col(6+component) = Eigen::Map3x<DataType>(dphi<2>(x).data());
        tmp.col(9+component) = Eigen::Map3x<DataType>(dphi<3>(x).data());
        
        return tmp;
        
    }
    
    inline Eigen::Vector3x<DataType> x(double alphaX, double alphaY, double alphaZ) const {
        Eigen::Vector3x<DataType> lambda;
        lambda << alphaX, alphaY, alphaZ;
        return m_T.inverse()*lambda + m_x3;
    }
    
    inline std::array<DOFBase<DataType,0>*,4>  getQDofs()
    {
        return m_qDofs;
    }
    
    inline double volume() { return (1.0/6.0)*(1.0/m_T.determinant()); }
    
    constexpr unsigned int getNumVerts() { return 4; }
    
    inline Eigen::Matrix<DataType, 3,3> getInvRefShapeMatrix(){
        return m_T;
    }
    
    
protected:
    
    Eigen::Matrix<DataType, 3,3> m_T;
    Eigen::Matrix<DataType, 3,1> m_x3; //maybe don't save this just replace with a pointer and a map
    
    std::array<DOFBase<DataType,0> *, 4> m_qDofs;
    std::array<DOFBase<DataType,1> *, 4> m_qDotDofs;
    
private:
    
};
