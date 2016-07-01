#define SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_CPP

#include "RigidMassComplianceEngine.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{
namespace component
{
namespace engine
{

SOFA_DECL_CLASS(RigidMassComplianceEngine)

int RigidMassComplianceEngineClass = sofa::core::RegisterObject("Generate rigid compliance from rigid mass")
#ifndef SOFA_FLOAT
        .add< RigidMassComplianceEngine<sofa::defaulttype::Rigid3dTypes> >() 
#endif
#ifndef SOFA_DOUBLE
        .add < RigidMassComplianceEngine<sofa::defaulttype::Rigid3fTypes> >()
#endif 
        ;

#ifndef SOFA_FLOAT
template class SOFA_CONSTRAINT_API RigidMassComplianceEngine< sofa::defaulttype::Rigid3dTypes >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_CONSTRAINT_API RigidMassComplianceEngine< sofa::defaulttype::Rigid3fTypes >;
#endif

}

}

}
