#include <iostream>
#include <Eigen/Dense>
#include <limits>

//Gradient descent
//INPUT:
//f - function object which returns the objective to be minimized. Must be of the form float f(Eigen::VectorXd &x)
//g - function object which computes the gradient of the objective being minimized. Must be of the form Eigen::VectorXd f(Eigen::VectorXd &g, Eigen::VectorXd &x). Upon return, g returns the gradient evaluated at x.
//stepSize - the step size for the gradient descent algorithm.
//maxSteps - maximum gradient descent steps to take.
//OUTPUT:
//the energy value and parameter values at which the optimization terminates
template<typename Objective, typename Gradient>
double gradientDescent(Eigen::VectorXd &x0,
                                                  Objective &f,
                                                  Gradient &g,
                                                  float stepSize,
                      unsigned int maxSteps = std::numeric_limits<int>::max());

//Back tracking linesearch
//INPUT:
// x0 - current point in search space
// dx - search direction
// f - objective function (same format as above)
//OUTPUT:
// x0 is set to the best point found in the search direction dx
template<typename Objective>
void backtracking(Eigen::VectorXd &x0, Eigen::VectorXd &dx, Objective &f);

//Gradient Descent with Back Tracking
//INPUT and OUTPUT are the same as for gradient
template<typename Objective, typename Jacobian>
double gradientDescentWithBacktracking(Eigen::VectorXd &x0,
                                       Objective &f,
                                       Jacobian &g,
                                       unsigned int maxSteps = std::numeric_limits<int>::max());

//compute hessian using finite differences
//INPUT:
//  H - hessian matrix of f, computed at point x0
//  x0 - point in search space
//  f - objective function (same format as methods above)
//  fd - the tolerance to use for the finite difference calculation
//OUTPUT:
//  H - contains the hessian of f computed at x0.
template<typename Objective>
void hessian(Eigen::MatrixXd &H, Eigen::VectorXd &x0, Objective &f, double fd=1e-1);

//Newton's Method with Back Tracking
//INPUT and OUTPUT are the same as the gradient descent methods above
template<typename Objective, typename Jacobian>
double newtonWithBacktracking(Eigen::VectorXd &x0,
                                       Objective &f,
                                       Jacobian &g,
                                       unsigned int maxSteps = std::numeric_limits<int>::max());


