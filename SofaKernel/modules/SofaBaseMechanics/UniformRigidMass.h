#ifndef SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
#define SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
#include "config.h"

#include <sofa/core/behavior/Mass.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/core/objectmodel/Base.h>

namespace sofa
{
namespace component
{
namespace mass
{

template < class RigidDataTypes > 
class UniformRigidMass : public virtual sofa::core::behavior::Mass< RigidDataTypes >
{
public:
    SOFA_CLASS( SOFA_TEMPLATE( UniformRigidMass, RigidDataTypes ), 
                 SOFA_TEMPLATE( sofa::core::behavior::Mass, RigidDataTypes ) );
    
    enum { spatial_dimensions = RigidDataTypes::spatial_dimensions };

    typedef Inherit1 Inherit;

    typedef typename Inherit::DataVecCoord DataVecCoord;
    typedef typename Inherit::DataVecDeriv DataVecDeriv;
    typedef typename Inherit::VecCoord     VecCoord;
    typedef typename Inherit::VecDeriv     VecDeriv;

    typedef typename RigidDataTypes::Real        Real;
    typedef typename RigidDataTypes::Coord       RigidCoord;
    typedef typename RigidDataTypes::Deriv       RigidDeriv;
    typedef typename RigidDataTypes::MatrixDeriv RigidMatrixDeriv;

    typedef typename sofa::defaulttype::RigidMass< spatial_dimensions, Real > RigidMass;


    sofa::Data< sofa::helper::vector<RigidMass> > d_mass;
    sofa::Data< bool  >      d_useGyroscopicExplicit;
    sofa::Data< Real  >      d_maxGyroscopicForce;
    sofa::Data< float >      d_drawAxisFactor;

    virtual void init();

    virtual void reinit();

    /// Mass API 

    virtual void addForce(const sofa::core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecCoord& d_x, const DataVecDeriv& d_v);

    virtual void addMDx( const sofa::core::MechanicalParams* mparams, DataVecDeriv& d_f, const DataVecDeriv& d_dx, double factor );

    virtual void addMToMatrix(const core::MechanicalParams *mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix);

    virtual void accFromF(const core::MechanicalParams* mparams, DataVecDeriv& a, const DataVecDeriv& f);

    //template<class MatrixWriter>
    //void addMToMatrixT(const core::MechanicalParams* mparams, MatrixWriter m);

    virtual SReal getKineticEnergy( const sofa::core::MechanicalParams* mparams, const DataVecDeriv& v) const
    {
        return SReal(0);
    }

    virtual SReal getPotentialEnergy( const sofa::core::MechanicalParams* mparams, const DataVecCoord& x ) const
    {
        return SReal(0);
    }


    virtual void draw(const sofa::core::visual::VisualParams* vparams);

protected:

    UniformRigidMass();

};

/// Convert to global coordinates the local matrix using the given orientation quaternion.
/// local is a diagonal matrix ( either the local inertia mass matrix, or its inverse )
template< typename TQuat , typename TReal >
sofa::defaulttype::Mat<3,3,TReal> localToGlobal(const TQuat& orientation, const sofa::defaulttype::Mat<3,3,TReal>& local )
{
    sofa::defaulttype::Mat<3,3,TReal> rotation(sofa::defaulttype::NOINIT);
    orientation.toMatrix(rotation);

    const typename sofa::defaulttype::Mat<3,3,TReal>::Line diag = sofa::defaulttype::diagonal(local);
    sofa::defaulttype::Mat<3,3,TReal> global = rotation.multDiagonal( diag ) * rotation.transposed();

    return global;
}

template< typename TRigidCoord, typename TRigidMass >
typename TRigidMass::Mat3x3 computeInertiaMassMatrixWorld( const TRigidCoord& x, const TRigidMass& mass)
{
    const typename TRigidMass::Mat3x3 inertiaMassMatrixLocal = mass.inertiaMatrix * mass.mass;
    return localToGlobal( x.getOrientation(), inertiaMassMatrixLocal );
}

template< typename TRigidCoord, typename TRigidMass >
typename TRigidMass::Mat3x3 computeInvInertiaMassMatrixWorld( const TRigidCoord& x, const TRigidMass& mass)
{
    const typename TRigidMass::Mat3x3 invInertiaMassMatrixLocal = mass.invInertiaMatrix / mass.mass;
    return localToGlobal( x.getOrientation(), invInertiaMassMatrixLocal );
}

/// returns an explicit computation of the gyroscopic force.
/// in an explicit setting the gyroscopic force is gf = w x I.w  where :
/// - w is the angular velocity
/// - I is the inertia mass matrix in world coordinates
template< typename TRigidCoord, typename TVec3, typename TRigidMass >
TVec3 computeGyroscopicForceExplicit(const TRigidCoord& x, const TVec3& w, const TRigidMass&  mass )
{
    const typename TRigidMass::Mat3x3  intertiaMassMatrixWorld = computeInertiaMassMatrixWorld(x,mass);
    const TVec3 Iw   = intertiaMassMatrixWorld * w;
    const TVec3 wxIw = sofa::defaulttype::cross( w, Iw );
    return wxIw;
}

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BASE_MECHANICS_API UniformRigidMass<sofa::defaulttype::Rigid3dTypes>;
#endif 
#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_MECHANICS_API UniformRigidMass<sofa::defaulttype::Rigid3fTypes>;
#endif 
#endif 

}

}

}

#endif //SOFA_COMPONENT_MASS_UNIFORMRIGIDMASS_H
