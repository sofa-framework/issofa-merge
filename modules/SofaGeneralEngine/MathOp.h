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
#ifndef SOFA_COMPONENT_ENGINE_MATHOP_H
#define SOFA_COMPONENT_ENGINE_MATHOP_H
#include "config.h"

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/OptionsGroup.h>

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * Apply a math operation to combine several inputs
 */
template <class VecT>
class MathOp : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MathOp,VecT),core::DataEngine);
    typedef VecT VecValue;
    typedef typename VecValue::value_type Value;

protected:
    MathOp();

    ~MathOp();
public:
    /// Parse the given description to assign values to this object's fields and potentially other parameters
    void parse ( sofa::core::objectmodel::BaseObjectDescription* arg );

    /// Assign the field values stored in the given map of name -> value pairs
    void parseFields ( const std::map<std::string,std::string*>& str );

    void init();

    void reinit();

    void handleEvent(core::objectmodel::Event* e);

    void update();

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const MathOp<VecT>* = NULL)
    {
        return Data<Value>::templateName();
    }

    Data<unsigned int> f_nbInputs;
    helper::vector<Data<VecValue>*> vf_inputs;
    sofa::core::objectmodel::Data< sofa::helper::OptionsGroup > f_op;
    Data<VecValue> f_output;

protected:
    void createInputs(int nb = -1);
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_MATHOP_CPP)

extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<int> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<bool> >;
#ifndef SOFA_FLOAT
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<double> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<defaulttype::Vec2d> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<defaulttype::Vec3d> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid2dTypes::VecCoord >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid2dTypes::VecDeriv >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid3dTypes::VecCoord >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid3dTypes::VecDeriv >;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<float> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<defaulttype::Vec2f> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< helper::vector<defaulttype::Vec3f> >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid2fTypes::VecCoord >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid2fTypes::VecDeriv >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid3fTypes::VecCoord >;
extern template class SOFA_GENERAL_ENGINE_API MathOp< defaulttype::Rigid3fTypes::VecDeriv >;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
