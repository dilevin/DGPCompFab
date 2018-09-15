//
//  Shape.h
//  CompFabA1
//
//  Created by David Levin on 8/3/18.
//

#ifndef Shape_h
#define Shape_h

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<string>
#include <tuple>
#include <vector>
#include <map>
#include <unordered_map>
#include <libfive/tree/tree.hpp>
#include <libfive/eval/eval_array.hpp>
#include <libfive/render/brep/mesh.hpp>
#include <libfive/render/brep/xtree_pool.hpp>
#include <libfive/render/brep/region.hpp>
#include <libfive/eval/eval_jacobian.hpp> //get jacobian of implicit function

//Global list of Trees and names, values
//shape just needs to know the variable name then it can duplicate ids and get values from main list
//Define a Variable Struct which serves as  the internal representation of a
//scalar variable
class ShapeVar {
    
public:
    
    //releate strings to a vector of Trees which represent parameters. Using vectors allows for
    //multivariate parameters.
    static std::unordered_map<std::string, std::pair<std::vector<Kernel::Tree>, std::vector<float>>> s_varTrees;
    
    //scalar constructor
    ShapeVar(std::string name, float x);
    
    //mutivariante constructor
    ShapeVar(std::string name, const Eigen::VectorXf &x);
    
    inline const std::string & name() const { return m_varName; }
    //you'll need operator overloads to make this convenient
    
    inline const Kernel::Tree & tree(unsigned int i=0) const {
        return (s_varTrees[m_varName].first[i]);
    }
    
    inline Kernel::Tree & tree(unsigned int i=0) {
        return (s_varTrees[m_varName].first[i]);
    }
    
    inline const Kernel::Tree & operator()(unsigned int i=0) const {
        return tree(i);
    }
    
    inline Kernel::Tree & operator()(unsigned int i=0) {
        return tree(i);
    }
    
    inline float & operator[](unsigned int i) {
        return s_varTrees[m_varName].second[i];
    }
    
    static inline  std::unordered_map<std::string, std::pair<std::vector<Kernel::Tree>, std::vector<float>>> & allVars() { return s_varTrees; }
    
    //get total number of scalar vars in the system
    static inline unsigned int numScalarVars() { return m_numScalarVars; }
    
protected:
    
    //name of this variable
    std::string m_varName;
    static unsigned int m_numScalarVars;
    
private:
};

#define OP_UNARY_VAR(OP)       inline Kernel::Tree OP(const ShapeVar& a) {\
                                    return OP(a.tree(0));\
                                }


#define OP_BINARY_VAR(OP)       inline Kernel::Tree OP(const Kernel::Tree& a, const ShapeVar& b) {\
                                    return OP(a,b.tree(0));\
                                }\
                                inline Kernel::Tree OP(const ShapeVar& a, const Kernel::Tree& b) {\
                                    return OP(a.tree(0),b);\
                                }\
                                inline Kernel::Tree OP(const ShapeVar& a, const ShapeVar& b) {\
                                    return OP(a.tree(0),b.tree(0));\
                                }

OP_UNARY_VAR(square);
OP_UNARY_VAR(sqrt);
OP_UNARY_VAR(abs);
OP_UNARY_VAR(sin);
OP_UNARY_VAR(cos);
OP_UNARY_VAR(tan);
OP_UNARY_VAR(asin);
OP_UNARY_VAR(acos);
OP_UNARY_VAR(atan);
OP_UNARY_VAR(log);
OP_UNARY_VAR(exp);

OP_BINARY_VAR(operator+);
OP_BINARY_VAR(operator*);
OP_BINARY_VAR(min);
OP_BINARY_VAR(max);
OP_BINARY_VAR(operator-);
OP_BINARY_VAR(operator/);
OP_BINARY_VAR(atan2);
OP_BINARY_VAR(pow);
OP_BINARY_VAR(nth_root);
OP_BINARY_VAR(mod);
OP_BINARY_VAR(nanfill);
OP_BINARY_VAR(compare);

//This class encapulates a paramertric shape that is differentiable with resppect to its design paramters
class Shape {
public:
    
    Shape();
    
    inline Shape(const Kernel::Tree &tree) : m_shapeTree(0) { m_shapeTree = tree; }
    
    void registerVariable(const ShapeVar &var); //register variable in evaluation lists
    
    inline void operator=(const Kernel::Tree &tree) { m_shapeTree = tree; }
    
    inline const Kernel::Tree & operator*() const { return m_shapeTree; }

    //update variables from central variable storage
    void updateVars();
    
    //get the variable map for this object.
    inline  const std::map<Kernel::Tree::Id, float> & vars() { return m_vars; }
    
    double value(const Eigen::Vector3f &x);
    
    void jacobian(Eigen::VectorXd &Jout, const Eigen::Vector3f &x);
    
protected:

    
    std::map<Kernel::Tree::Id, std::pair<std::string, unsigned int>> m_varName;  //use this to lookup values from global variable store
    std::map<Kernel::Tree::Id, float> m_vars; //libfive takes this structure as a parameter for its evaluators
    
    //The tree for this entire shape
    Kernel::Tree m_shapeTree;
    
private:
    
};

std::unique_ptr<Kernel::Mesh> drawMesh(Shape &toDraw, Kernel::Region<3> &region, float resolution);

#endif /* Shape_h */
