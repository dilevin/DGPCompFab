template<typename Objective, typename Jacobian>
double gradientDescent(Eigen::VectorXd &x0, Objective &f, Jacobian &g, float stepSize, unsigned int maxSteps) {
    
    return 0.0;
}

template<typename Objective, typename Jacobian>
double gradientDescentWithBacktracking(Eigen::VectorXd &x0, Objective &f, Jacobian &g, unsigned int maxSteps) {

    return 0.0;
}

template<typename Objective>
void backtracking(Eigen::VectorXd &x0, Eigen::VectorXd &dx, Objective &f) {
    
}

template<typename Objective>
void hessian(Eigen::MatrixXd &H, Eigen::VectorXd &x0, Objective &f, double fd) {
    
    
}

template<typename Objective, typename Jacobian>
double newtonWithBacktracking(Eigen::VectorXd &x0, Objective &f, Jacobian &g, unsigned int maxSteps) {
   
    return 0.0;
}


