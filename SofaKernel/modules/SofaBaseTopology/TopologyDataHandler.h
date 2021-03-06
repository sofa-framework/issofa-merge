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
#ifndef SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_H
#define SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_H
#include "config.h"

#include <sofa/core/topology/TopologyElementHandler.h>
#include <sofa/core/topology/BaseTopologyData.h>


namespace sofa
{

namespace component
{

namespace topology
{


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Generic Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Helper method to initialize a value_type stored in a topology-indexed Data.
/// Can be specialized for types without default constructors
template< class value_type >
inline void TopologyDataHandler_clear(value_type& v) { v = value_type(); }

/// Helper method to resize a container_type stored in a topology-indexed Data.
/// Can be specialized for types without default constructors
template< class container_type >
inline void TopologyDataHandler_resize(container_type& data, unsigned int n) { data.resize(n); }

/** \brief A class for storing Edge related data. Automatically manages topology changes.
*
* This class is a wrapper of class helper::vector that is made to take care transparently of all topology changes that might
* happen (non exhaustive list: elements added, removed, fused, renumbered).
*/

template< class TopologyElementType, class VecT>
class TopologyDataHandler : public sofa::core::topology::TopologyElementHandler< TopologyElementType >
{
public:
    typedef VecT container_type;
    typedef typename container_type::value_type value_type;

    /// size_type
    typedef typename container_type::size_type size_type;
    /// reference to a value (read-write)
    typedef typename container_type::reference reference;
    /// const reference to a value (read only)
    typedef typename container_type::const_reference const_reference;
    /// const iterator
    typedef typename container_type::const_iterator const_iterator;

    typedef sofa::core::topology::TopologyElementHandler< TopologyElementType > Inherit;
    typedef typename Inherit::AncestorElem AncestorElem;

protected:
    sofa::core::topology::BaseTopologyData <VecT>* m_topologyData;
	value_type* m_defaultValue; // default value when adding an element (or NULL if not specified, in which case TopologyDataHandler_clear() is used)

public:
    /// constructor without default value
    TopologyDataHandler(sofa::core::topology::BaseTopologyData <VecT>* _topologyData)
        :sofa::core::topology::TopologyElementHandler < TopologyElementType >()
        , m_topologyData(_topologyData), m_defaultValue(NULL) {}

    /// constructor with default value
    TopologyDataHandler(sofa::core::topology::BaseTopologyData <VecT>* _topologyData,
                        value_type defaultValue)
        :sofa::core::topology::TopologyElementHandler < TopologyElementType >()
        , m_topologyData(_topologyData), m_defaultValue(new value_type(defaultValue)) {}

    /// destructor
    ~TopologyDataHandler()
    {
        if (m_defaultValue)
        {
            delete m_defaultValue;
        }
    }

    bool isTopologyDataRegistered()
    {
        if(m_topologyData) return true;
        else return false;
    }

    /** Public fonction to apply creation and destruction functions */
    /// Apply removing current elementType elements
    virtual void applyDestroyFunction(unsigned int, value_type& ) {}

    /// Apply adding current elementType elements
    virtual void applyCreateFunction(unsigned int, value_type& t,
            const sofa::helper::vector< unsigned int > &,
            const sofa::helper::vector< double > &)
    {
        if (m_defaultValue)
        {
            t = *m_defaultValue;
        }
        else
        {
            TopologyDataHandler_clear(t);
        }
    }

    /// WARNING NEED TO UNIFY THIS
    /// Apply adding current elementType elements
    virtual void applyCreateFunction(unsigned int i, value_type&t , const TopologyElementType& ,
            const sofa::helper::vector< unsigned int > &ancestors,
            const sofa::helper::vector< double > &coefs)
    {
        applyCreateFunction(i, t, ancestors, coefs);
    }

    virtual void applyCreateFunction(unsigned int i, value_type&t , const TopologyElementType& e,
            const sofa::helper::vector< unsigned int > &ancestors,
            const sofa::helper::vector< double > &coefs,
            const AncestorElem* /*ancestorElem*/)
    {
        applyCreateFunction(i, t, e, ancestors, coefs);
    }
	// update the default value used during creation
	void setDefaultValue(const value_type &v)
    {
        if (m_defaultValue)
        {
            *m_defaultValue = v;
        }
        else
        {
            m_defaultValue = new value_type(v);
        }
	}

protected:
    /// Swaps values at indices i1 and i2.
    virtual void swap( unsigned int i1, unsigned int i2 );

    /// Add some values. Values are added at the end of the vector.
    /// This (new) version gives more information for element indices and ancestry
    virtual void add( const sofa::helper::vector<unsigned int> & index,
            const sofa::helper::vector< TopologyElementType >& elems,
            const sofa::helper::vector< sofa::helper::vector< unsigned int > > &ancestors,
            const sofa::helper::vector< sofa::helper::vector< double > >& coefs,
            const sofa::helper::vector< AncestorElem >& ancestorElems);

    /// Remove the values corresponding to the elements removed.
    virtual void remove( const sofa::helper::vector<unsigned int> &index );

    /// Reorder the values.
    virtual void renumber( const sofa::helper::vector<unsigned int> &index );

    /// Move a list of points
    virtual void move( const sofa::helper::vector<unsigned int> &indexList,
            const sofa::helper::vector< sofa::helper::vector< unsigned int > >& ancestors,
            const sofa::helper::vector< sofa::helper::vector< double > >& coefs);

    /// Add Element after a displacement of vertices, ie. add element based on previous position topology revision.
    virtual void addOnMovedPosition(const sofa::helper::vector<unsigned int> &indexList,
            const sofa::helper::vector< TopologyElementType > & elems);

    /// Remove Element after a displacement of vertices, ie. add element based on previous position topology revision.
    virtual void removeOnMovedPosition(const sofa::helper::vector<unsigned int> &indices);


};


} // namespace topology

} // namespace component

} // namespace sofa


#endif // SOFA_COMPONENT_TOPOLOGY_TOPOLOGYDATAHANDLER_H
