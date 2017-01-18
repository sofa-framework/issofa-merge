/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_INTERACTIONFORCEFIELD_PLANEFORCEFIELD_H
#define SOFA_COMPONENT_INTERACTIONFORCEFIELD_PLANEFORCEFIELD_H
#include "config.h"

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

/// This class can be overridden if needed for additionnal storage within
/// template specializations.
template<class DataTypes>
class PlaneForceFieldInternalData
{
public:
};

///
/// @class PlaneForceField
/// A plane is cutting the space in two half spaces. This component generate a force preventing the
/// object to cross the plane. The plane is defined by its normal and by the amount of displacement
/// along this normal.
template<class DataTypes>
class PlaneForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PlaneForceField, DataTypes),
               SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));
    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord   VecCoord;
    typedef typename DataTypes::VecDeriv   VecDeriv;
    typedef typename DataTypes::Coord      Coord;
    typedef typename DataTypes::Deriv      Deriv;
    typedef typename DataTypes::VecReal    VecReal;
    typedef typename DataTypes::Real       Real;
    typedef typename DataTypes::CPos       CPos;
    typedef typename DataTypes::DPos       DPos;
    typedef typename Inherit::DataVecCoord DataVecCoord;
    typedef typename Inherit::DataVecDeriv DataVecDeriv;

    typedef sofa::SingleLink< PlaneForceField<DataTypes>, sofa::core::topology::BaseMeshTopology, 
        sofa::BaseLink::FLAG_STRONGLINK | sofa::BaseLink::FLAG_STOREPATH > BaseMeshTopologyLink;

protected:

    struct PlaneContact
    {
        unsigned int index;
        Real         d;

        PlaneContact(unsigned int index, Real d)
        :index(index)
        ,d(d)
        {
        }

        PlaneContact()
        :index(sofa::core::topology::BaseMeshTopology::InvalidID)
        ,d(0)
        {
        }

        inline friend std::ostream& operator<< ( std::ostream& out, const PlaneContact& planeContact )
        {
            out << "PLANECONTACT"
                << " index= " << planeContact.index
                << " d= "     << planeContact.d 
                << " END\n";
            return out;
        }

        inline friend std::istream& operator>> ( std::istream& in, PlaneContact& planeContact )
        {
            std::string str;
            if ((in >> str) && (str == "PLANECONTACT"))
            {
                while (in >> str)
                {
                    if (str == "END")
                    {
                        break;
                    }
                    else if( str=="index=" )
                    {
                        in >> planeContact.index;
                    }
                    else if( str=="d=" )
                    {
                        in >> planeContact.d;
                    }
                }
            }
            return in;
        }
    };



    PlaneForceFieldInternalData<DataTypes> m_data;

public:

    Data<DPos>                                        d_planeNormal;
    Data<VecReal>                                     d_planeD;
    Data<Real>                                        d_stiffness;
    Data<Real>                                        d_damping;
    Data<Real>                                        d_maxForce;
    Data<sofa::helper::vector< unsigned > >           d_indices; //< BaseMeshTopology should provide a way to get the PointID from the array of points.

    Data<bool>               d_drawIsEnabled;
    Data<defaulttype::Vec3f> d_drawColor;
    Data<Real>               d_drawSize;
    /// option bilateral : if true, the force field is applied on both side of the plane
    Data<bool>                                        d_bilateral;
    
    sofa::Data< sofa::helper::vector<PlaneContact > > d_contacts;

protected:
    PlaneForceField() ;

public:

    void setPlane(const Deriv& normal, Real d);
    void setMState(  core::behavior::MechanicalState<DataTypes>* mstate ) 
    { 
        this->mstate = mstate; 
    }

    void setStiffness(Real stiff) { d_stiffness.setValue( stiff ); }
    Real getStiffness() const { return d_stiffness.getValue(); }

    void setDamping(Real damp){ d_damping.setValue( damp ); }
    Real getDamping() const { return d_damping.getValue(); }

    void setDrawColor(const defaulttype::Vec3f& newvalue){ d_drawColor.setValue(newvalue); }
    const defaulttype::Vec3f& getDrawColor() const { return d_drawColor.getValue(); }

    //TODO(dmarchal): do we really need a rotate operation into a plan class ?
    void rotate( Deriv axe, Real angle ); // around the origin (0,0,0)

    /// Inherited from ForceField.
    virtual void init();
    virtual void addForce(const core::MechanicalParams* mparams,
                          DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v);
    virtual void addDForce(const core::MechanicalParams* mparams,
                           DataVecDeriv& df, const DataVecDeriv& dx);
    virtual SReal getPotentialEnergy(const core::MechanicalParams* /*mparams*/,
                                     const DataVecCoord&  /* x */) const;
    virtual void updateStiffness( const VecCoord& x );
    virtual void addKToMatrix(const core::MechanicalParams*
                              mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix );

    void draw(const core::visual::VisualParams* vparams);
    void drawPlane(const core::visual::VisualParams*, float size=0.0f);
    void computeBBox(const core::ExecParams *, bool onlyVisible=false);

};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_INTERACTIONFORCEFIELD_PLANEFORCEFIELD_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec3dTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec2dTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec1dTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec6dTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec3fTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec2fTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec1fTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Vec6fTypes>;
extern template class SOFA_BOUNDARY_CONDITION_API PlaneForceField<defaulttype::Rigid3fTypes>;
#endif
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
