#include <GaussIncludes.h>
#include <FEMIncludes.h>
#include "QuadratureLinearElasticityA4.h"
#include "ShapeFunctionTetLinearA4.h"
//convenience definition for shared potential and kinetic energy quadrature rules
template<   typename DataType,
template<typename Type, typename Energy> class QuadratureRule,
template<typename Type> class ShapeFunction >
using ElementStatic = Gauss::FEM::ElementBase<DataType, 4,
    Gauss::FEM::QuadratureExact, QuadratureRule, Gauss::FEM::QuadratureExact,
    Gauss::FEM::EnergyKineticNonLumped, Gauss::FEM::EnergyLinearElasticity,
    Gauss::FEM::BodyForceGravity, ShapeFunction>;
