//
//  interopUtils.cpp
//  CompFabA1
//
//  Created by David Levin on 7/22/18.
//
#include "../include/interopUtils.h"

void fiveToIGL(Eigen::MatrixXd &V, Eigen::MatrixXi &F, const Kernel::BRep<3> &from) {
    
    size_t numVertices = from.verts.size();
    size_t numFaces  = from.branes.size();
    
    V.resize(numVertices, 3);
    F.resize(numFaces,3);
    
    for(size_t iv=0; iv<numVertices; ++iv) {
        V(iv,0) = from.verts[iv][0]; V(iv,1) = from.verts[iv][1]; V(iv,2) = from.verts[iv][2];
    }
    
    for(size_t it=0; it<numFaces; ++it) {
        F(it,0) = from.branes[it][0]; F(it,1) = from.branes[it][1]; F(it,2) = from.branes[it][2];
    }
}
