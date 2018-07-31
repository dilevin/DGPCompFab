//
//  interopUtils.h
//  CompFabA1
//
//  Created by David Levin on 7/22/18.
//

#ifndef interopUtils_h
#define interopUtils_h

#include "libfive/render/brep/mesh.hpp"
#include "libfive/render/brep/xtree_pool.hpp"
#include "libfive/render/brep/region.hpp"

void fiveToIGL(Eigen::MatrixXd &V, Eigen::MatrixXi &F, const Kernel::BRep<3> &from);

#endif /* interopUtils_h */
