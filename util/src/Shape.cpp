//
//  Shapes.cpp
//  CompFabA2
//
//  Created by David Levin on 8/3/18.
//

#include <Shape.h>
#include <libfive/tree/tree.hpp>

std::unordered_map<std::string, std::pair<std::vector<Kernel::Tree>, std::vector<float>>> ShapeVar::s_varTrees;
unsigned int ShapeVar::m_numScalarVars = 0;

//scalar constructor
ShapeVar::ShapeVar(std::string name, float x) : m_varName(name) {
    //std::unordered_map<std::string, std::pair<std::vector<Kernel::Tree *>, std::vector<float>> m_varTrees;
    std::vector<float> val;
    std::vector<Kernel::Tree> tree;
    tree.push_back(Kernel::Tree(Kernel::Opcode::VAR_FREE));
    val.push_back(x);
    s_varTrees.insert({name, {tree, val}});
    m_numScalarVars++;
}

//mutivariante constructor
ShapeVar::ShapeVar(std::string name, const Eigen::VectorXf &x) : m_varName(name) {
    
    std::vector<float> val;
    std::vector<Kernel::Tree> tree;
    for(unsigned int ii=0; ii<x.size(); ++ii) {
        tree.push_back(Kernel::Tree::var());
        val.push_back(x[ii]);
        m_numScalarVars++;
    }
    
    s_varTrees.insert({name, {tree, val}});
}

Shape::Shape() : m_shapeTree(0) {
    
}

void Shape::registerVariable(const ShapeVar &var) {
    
    std::pair<std::vector<Kernel::Tree>, std::vector<float>> &vals = ShapeVar::s_varTrees[var.name()];
    
    if(vals.first.size() != vals.second.size()) {
        std::cout<<"Error in registerVariable: The number of variables is different from the number of values \n";
        return;
    }
    
    for(unsigned int ii=0; ii<vals.first.size(); ++ii) {
        m_varName.insert({vals.first[ii].id(), {var.name(), ii}});
        m_vars.insert({vals.first[ii].id(), vals.second[ii]});
    }
    
}

void Shape::updateVars() {
    
    auto variableMap = ShapeVar::s_varTrees;
    
    for(auto &var : m_vars) {
        auto &x = m_varName[var.first];
        var.second = variableMap[x.first].second[x.second];
    }

}

double Shape::value(const Eigen::Vector3f &x) {
    updateVars();
    auto e = new Kernel::JacobianEvaluator(std::make_shared<Kernel::Deck>(m_shapeTree), vars());
    return static_cast<double>(e->eval(x)); //value of distance function at x
    delete e;
}

void Shape::jacobian(Eigen::VectorXd &Jout, const Eigen::Vector3f &x) {
    updateVars();
    auto e = new Kernel::JacobianEvaluator(std::make_shared<Kernel::Deck>(m_shapeTree), vars());
    
    auto gradient = e->gradient(x);
    
    //pull everything into eigen matrix
    auto &allVars = ShapeVar::allVars();
    Jout.resize(ShapeVar::numScalarVars(),1);
    Jout.setZero(); //zero everything out
    
    unsigned int ii=0;
    for(auto &var : allVars) {
        //check if this shape needs this scalar variable
        for(unsigned int jj=0; jj<var.second.first.size();++jj) {
            //does m_vars constain this scalar variable
            if(m_vars.find(var.second.first[jj].id()) != m_vars.end()) {
                Jout[ii] = static_cast<double>(gradient[var.second.first[jj].id()]);
                ++ii;
            }
        }
        
    }
    
    delete e;
}


std::unique_ptr<Kernel::Mesh> drawMesh(Shape &toDraw, Kernel::Region<3> &region, float resolution) {
    toDraw.updateVars();
    std::atomic_bool cancel;
    return Kernel::Mesh::render(*toDraw, toDraw.vars(), region, resolution, resolution, 1, cancel);
}

