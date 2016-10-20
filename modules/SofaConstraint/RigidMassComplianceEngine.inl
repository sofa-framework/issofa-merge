#ifndef SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_INL
#define SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_INL

#include "RigidMassComplianceEngine.h"

#include <SofaBaseMechanics/UniformRigidMass.h>

namespace sofa
{
namespace component
{
namespace engine
{

template< class TRigidDataTypes>
RigidMassComplianceEngine<TRigidDataTypes>::RigidMassComplianceEngine()
:d_rigidPosition(initData(&d_rigidPosition,"rigidPosition","INPUT:  />The rigid positions to use to compute the inertia mass matrix"))
,d_rigidMass(initData(&d_rigidMass,"rigidMass","INPUT: rigid mass vector"))
,d_rigidCompliance(initData(&d_rigidCompliance,"rigidCompliance","OUTPUT: rigid compliance vector"))
,d_rigidComplianceUnzipped(initData(&d_rigidComplianceUnzipped,"rigidComplianceUnzipped","OUTPUT: rigid compliance as a a vector of scalars"))
{
    addInput(&d_rigidPosition);
    addInput(&d_rigidMass);
    addOutput(&d_rigidCompliance);
    addOutput(&d_rigidComplianceUnzipped);
}

template< class TRigidDataTypes>
void RigidMassComplianceEngine<TRigidDataTypes>::init()
{
    this->setDirtyValue();
}

template< class TRigidDataTypes>
void RigidMassComplianceEngine<TRigidDataTypes>::update()
{
    const Real h  = Real( this->getContext()->getDt() );
    const Real h2 = h*h; 

    this->cleanDirty();

    sofa::helper::ReadAccessor < sofa::Data< TRigidVecCoord  > > rigidPosition = d_rigidPosition;
    
    if(!rigidPosition.empty() )
    {
        sofa::helper::ReadAccessor < sofa::Data< sofa::helper::vector<TRigidMass> > > rigidMass            = d_rigidMass;
        sofa::helper::WriteAccessor< sofa::Data< sofa::helper::vector<RigidCompliance> > > rigidCompliance = d_rigidCompliance;
        sofa::helper::WriteAccessor< sofa::Data< VecReal > > rigidComplianceUnzipped = d_rigidComplianceUnzipped; 

        const std::size_t size = rigidMass.size() != rigidPosition.size() ? std::min(rigidMass.size(),std::size_t(1) ) // a single mass for all dofs 
            : rigidPosition.size();
        rigidCompliance.resize( size );

        for(std::size_t i=0;i< size; ++i)
        {
            typename TRigidMass::Mat3x3 invInertiaGlobal = sofa::component::mass::computeInvInertiaMassMatrixWorld( rigidPosition[i], rigidMass[i] );
            rigidCompliance[i] = rigidComplianceFromInertiaMassMatrix( invInertiaGlobal, rigidMass[i].mass, h2 ); 
        }

        rigidComplianceUnzipped.clear();
        rigidComplianceUnzipped.reserve( RigidCompliance::total_size * rigidCompliance.size() );

        for(std::size_t i=0;i<rigidCompliance.size();++i)
        {
            std::copy( rigidCompliance[i].begin(), rigidCompliance[i].end(), std::back_inserter( rigidComplianceUnzipped ) );
        }

    }
}

template< class TRigidDataTypes>
typename RigidMassComplianceEngine<TRigidDataTypes>::RigidCompliance
RigidMassComplianceEngine<TRigidDataTypes>::rigidComplianceFromInertiaMassMatrix( const typename TRigidMass::Mat3x3& invInertiaGlobal, Real mass, Real /*h2*/ ) const
{

    RigidCompliance compliance(sofa::defaulttype::NOINIT);

    // W = dt^2 * invInertiaGlobal is only correct when solving the system in position
    //Thus, the formulation compliance = (h2 * invInertiaGlobal) should be deprecated 
    //Instead we just return the invert of inertiaMassMatrix and the contribution of the timestep is done within the constraintCorrection class
    //This way we ensure homogeneity when dealing with objects that use different constraintCorrection schemes.
    //WARNING: In this case when dealing with an uncoupledConstraintCorrection, the data useOdeIntegrationFactors must be set to true

    compliance[0] = (1 / mass);
    compliance[1] = invInertiaGlobal[0][0];
    compliance[2] = invInertiaGlobal[0][1];
    compliance[3] = invInertiaGlobal[0][2];
    compliance[4] = invInertiaGlobal[1][1];
    compliance[5] = invInertiaGlobal[1][2];
    compliance[6] = invInertiaGlobal[2][2];

    return compliance;
}

}

}

}

#endif // SOFACONSTRAINT_RIGIDMASSCOMPLIANCEENGINE_INL

