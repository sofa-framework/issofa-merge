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
#ifndef SOFA_HELPER_INTEGER_ID_H
#define SOFA_HELPER_INTEGER_ID_H

#include <sofa/helper/helper.h>
#include <sofa/helper/vector.h>
#include <sofa/helper/accessor.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <limits>

namespace sofa
{

namespace helper
{

typedef const char* (*integer_id_name)();

template < integer_id_name Name, typename Index = unsigned int, Index DefaultId = (Index)-1 >
class integer_id
{
public:
    typedef integer_id<Name, Index, DefaultId> Id;
    typedef Index index_type;
    typedef int sindex_type;
protected:
    index_type index;
public:
    static const char* getName() { return (*Name)(); }
    static index_type getDefaultId() { return DefaultId; }

    integer_id() : index(DefaultId) {}
    explicit integer_id(index_type i) : index(i) {}
    integer_id(const Id& i) : index(i.index) {}

    Id& operator=(const Id& i)
    {
        index = i.index;
        return *this;
    }

    index_type getId() const { return index; }
    void setId(index_type i) { index = i; }

    bool isValid() const
    {
        return index != DefaultId;
    }
    bool isValid(index_type size) const
    {
        return (unsigned)index < (unsigned)size;
    }

    bool operator==(const Id& a) const
    {
        return index == a.index;
    }

    bool operator==(const index_type& i) const
    {
        return index == i;
    }

    bool operator!=(const Id& a) const
    {
        return index != a.index;
    }

    bool operator!=(const index_type& i) const
    {
        return index != i;
    }

    bool operator<(const Id& a) const
    {
        return index < a.index;
    }

    bool operator<(const index_type& i) const
    {
        return index < i;
    }

    bool operator<=(const Id& a) const
    {
        return index <= a.index;
    }

    bool operator<=(const index_type& i) const
    {
        return index <= i;
    }

    bool operator>(const Id& a) const
    {
        return index > a.index;
    }

    bool operator>(const index_type& i) const
    {
        return index > i;
    }

    bool operator>=(const Id& a) const
    {
        return index >= a.index;
    }

    bool operator>=(const index_type& i) const
    {
        return index >= i;
    }

    template<typename int_type>
    Id operator+(int_type i) const
    {
        return Id(index + i);
    }

    template<typename int_type>
    Id& operator+=(int_type i)
    {
        index += i;
        return *this;
    }

    template<typename int_type>
    Id operator-(int_type i) const
    {
        return Id(index - i);
    }

    sindex_type operator-(const Id& i) const
    {
        return (sindex_type)(index - i.index);
    }
    
    template<typename int_type>
    Id& operator-=(int_type i)
    {
        index -= i;
        return *this;
    }
    
    Id& operator++()
    {
        ++index;
        return *this;
    }

    Id operator++(int)
    {
        Id old = *this;
        index++;
        return old;
    }
    
    Id& operator--()
    {
        --index;
        return *this;
    }
    
    Id operator--(int)
    {
        Id old = *this;
        index--;
        return old;
    }

    /// Output stream
    inline friend std::ostream& operator<< ( std::ostream& os, const Id& i )
    {
        return os << i.getId();
    }

    /// Input stream
    inline friend std::istream& operator>> ( std::istream& in, Id& i )
    {
        index_type v = i.getDefaultId();
        if (in >> v)
            i.setId(v);
        return in;
    }

};

void SOFA_HELPER_API vector_access_failure(const void* vec, unsigned size, unsigned i, const std::type_info& type, const char* tindex);

template <class T, class TIndex, bool CheckIndices =
#if !defined(NDEBUG) || defined(SOFA_CONTAINER_ACCESS_FAILURE)
    true
#else
    false
#endif
    , class MemoryManager = CPUMemoryManager<T> >
class vector_id : public vector<T, MemoryManager>
{
public:
    typedef vector<T, MemoryManager> Inherit;
    typedef T value_type;
    typedef TIndex index_type;
    typedef index_type ID;
    typedef typename Inherit::size_type size_type;
    typedef typename Inherit::reference reference;
    typedef typename Inherit::const_reference const_reference;
    typedef typename Inherit::iterator iterator;
    typedef typename Inherit::const_iterator const_iterator;

    template<class T2> struct rebind
    {
        typedef typename MemoryManager::template rebind<T2>::other MM2;
        typedef vector_id< T2, TIndex, CheckIndices, MM2 > other;
    };

    /// Basic constructor
    vector_id() : Inherit() {}
    /// Constructor
    vector_id(size_type n, const T& value): Inherit(n,value) {}
    /// Constructor
    vector_id(int n, const T& value): Inherit(n,value) {}
    /// Constructor
    vector_id(long n, const T& value): Inherit(n,value) {}
    /// Constructor
    explicit vector_id(size_type n): Inherit(n) {}
    /// Constructor
    vector_id(const std::vector<T>& x): Inherit(x) {}

#ifdef __STL_MEMBER_TEMPLATES
    /// Constructor
    template <class InputIterator>
    vector_id(InputIterator first, InputIterator last): Inherit(first,last) {}
#else /* __STL_MEMBER_TEMPLATES */
    /// Constructor
    vector_id(const_iterator first, const_iterator last): Inherit(first,last) {}
#endif /* __STL_MEMBER_TEMPLATES */

    /// Read/write random access, with explicit index_type
    reference at(index_type n)
    {
        if (CheckIndices)
        {
            if (!n.isValid(this->size()))
                vector_access_failure(this, this->size(), n.getId(), typeid(T), n.getName());
        }
        return *(this->begin() + n.getId());
    }

    /// Read-only random access, with explicit index_type
    const_reference at(index_type n) const
    {
        if (CheckIndices)
        {
            if (!n.isValid(this->size()))
                vector_access_failure(this, this->size(), n.getId(), typeid(T), n.getName());
        }
        return *(this->begin() + n.getId());
    }

    /// Read/write random access, with explicit index_type
    reference operator()(index_type n)
    {
        return at(n);
    }

    /// Read-only random access, with explicit index_type
    const_reference operator()(index_type n) const
    {
        return at(n);
    }

    /// Read/write random access, with explicit index_type
    reference operator[](index_type n)
    {
        return at(n);
    }

    /// Read-only random access
    const_reference operator[](index_type n) const
    {
        return at(n);
    }

    index_type push_back(const_reference v)
    {
        index_type i(this->size());
        Inherit::push_back(v);
        return i;
    }
protected:
    
    /// Read/write random access with regular index type, protected to force use of explicit index_type
    reference operator[](size_type n)
    {
        return at(index_type(n));
    }
    
    /// Read-only random access with regular index type, protected to force use of explicit index_type
    const_reference operator[](size_type n) const
    {
        return at(index_type(n));
    }

};


/// ReadAccessor implementation class for vector_id types
template<class T>
class ReadAccessorVectorId
{
public:
    typedef T container_type;
    typedef typename container_type::index_type index_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;
    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

protected:
    const container_type& vref;

public:
    ReadAccessorVectorId(const container_type& container) : vref(container) {}
    ~ReadAccessorVectorId() {}

    const container_type& ref() const { return vref; }

    bool empty() const { return vref.empty(); }
    size_type size() const { return vref.size(); }
    const_reference operator[](index_type i) const { return vref[i]; }
    const_reference operator()(index_type i) const { return vref(i); }

    const_iterator begin() const { return vref.begin(); }
    const_iterator end() const { return vref.end(); }

    inline friend std::ostream& operator<< ( std::ostream& os, const ReadAccessorVectorId<T>& vec )
    {
        return os << vec.vref;
    }

};

/// WriteAccessor implementation class for vector_id types
template<class T>
class WriteAccessorVectorId
{
public:
    typedef T container_type;
    typedef typename container_type::index_type index_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;
    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

protected:
    container_type& vref;

public:
    WriteAccessorVectorId(container_type& container) : vref(container) {}
    ~WriteAccessorVectorId() {}

    const container_type& ref() const { return vref; }
    container_type& wref() { return vref; }

    bool empty() const { return vref.empty(); }
    size_type size() const { return vref.size(); }

    const_reference operator[](index_type i) const { return vref[i]; }
    const_reference operator()(index_type i) const { return vref(i); }
    reference operator[](index_type i) { return vref[i]; }
    reference operator()(index_type i) { return vref(i); }

    const_iterator begin() const { return vref.begin(); }
    iterator begin() { return vref.begin(); }
    const_iterator end() const { return vref.end(); }
    iterator end() { return vref.end(); }

    void clear() { vref.clear(); }
    void resize(size_type s, bool /*init*/ = true) { vref.resize(s); }
    void reserve(size_type s) { vref.reserve(s); }
    index_type push_back(const_reference v) { return vref.push_back(v); }

    inline friend std::ostream& operator<< ( std::ostream& os, const WriteAccessorVectorId<T>& vec )
    {
        return os << vec.vref;
    }

    inline friend std::istream& operator>> ( std::istream& in, WriteAccessorVectorId<T>& vec )
    {
        return in >> vec.vref;
    }

};

// Support for vector_id

template <class T, class TIndex, bool CheckIndices, class MemoryManager>
class ReadAccessor< vector_id<T, TIndex, CheckIndices, MemoryManager> > : public ReadAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> >
{
public:
    typedef ReadAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> > Inherit;
    typedef typename Inherit::container_type container_type;
    ReadAccessor(const container_type& c) : Inherit(c) {}
};

template <class T, class TIndex, bool CheckIndices, class MemoryManager>
class WriteAccessor< vector_id<T, TIndex, CheckIndices, MemoryManager> > : public WriteAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> >
{
public:
    typedef WriteAccessorVectorId< vector_id<T, TIndex, CheckIndices, MemoryManager> > Inherit;
    typedef typename Inherit::container_type container_type;
    WriteAccessor(container_type& c) : Inherit(c) {}
};

template<class TDataType>
struct IntegerIdTypeInfo
{
    typedef TDataType DataType;
    typedef DataType BaseType;
    typedef typename DataType::index_type  index_type;
    typedef DataType ValueType;
    typedef long long ConvType; ///< preferred type for conversions (i.e. long long for integers, double for scalars)
    typedef IntegerIdTypeInfo<DataType> BaseTypeInfo;
    typedef IntegerIdTypeInfo<DataType> ValueTypeInfo;

    enum { ValidInfo = 1 }; ///< 1 if this type has valid infos
    enum { FixedSize = 1 }; ///< 1 if this type has a fixed size
    enum { ZeroConstructor = 1 }; ///< 1 if the constructor is equivalent to setting memory to 0
    enum { SimpleCopy = 1 }; ///< 1 if copying the data can be done with a memcpy
    enum { SimpleLayout = 1 }; ///< 1 if the layout in memory is simply N values of the same base type
    enum { Integer = 1 }; ///< 1 if this type uses integer values
    enum { Scalar = 0 }; ///< 1 if this type uses scalar values
    enum { Text = 0 }; ///< 1 if this type uses text values
    enum { CopyOnWrite = 0 }; ///< 1 if this type uses copy-on-write

    enum { Size = 1 }; ///< largest known fixed size for this type, as returned by size()
    static size_t size() { return 1; }

    static size_t size(const DataType& /*data*/) { return 1; }

    static void setSize(DataType& /*data*/, size_t /*size*/) {  }

    template <typename T>
    static void getValue(const DataType &data, size_t index, T& value)
    {
        if (index != 0) return;
        value = T(data.getId());
    }

    template<typename T>
    static void setValue(DataType &data, size_t index, const T& value)
    {
        if (index != 0) return;
        data.setId(index_type(value));
    }

    static void getValueString(const DataType &data, size_t index, std::string& value)
    {
        if (index != 0) return;
        std::ostringstream o; o << data; value = o.str();
    }

    static void setValueString(DataType &data, size_t index, const std::string& value)
    {
        if (index != 0) return;
        std::istringstream i(value); i >> data;
    }
};

template<class TDataType>
struct VectorIdTypeInfo
{
    typedef TDataType DataType;
    typedef typename DataType::size_type size_type;
    typedef typename DataType::index_type index_type;
    typedef typename DataType::value_type BaseType;
    typedef sofa::defaulttype::DataTypeInfo<BaseType> BaseTypeInfo;
    typedef typename BaseTypeInfo::ValueType ValueType;
    typedef sofa::defaulttype::DataTypeInfo<ValueType> ValueTypeInfo;

    enum { ValidInfo       = BaseTypeInfo::ValidInfo       }; ///< 1 if this type has valid infos
    enum { FixedSize       = 0                             }; ///< 1 if this type has a fixed size
    enum { ZeroConstructor = 0                             }; ///< 1 if the constructor is equivalent to setting memory to 0
    enum { SimpleCopy      = 0                             }; ///< 1 if copying the data can be done with a memcpy
    enum { SimpleLayout    = BaseTypeInfo::SimpleLayout    }; ///< 1 if the layout in memory is simply N values of the same base type
    enum { Integer         = BaseTypeInfo::Integer         }; ///< 1 if this type uses integer values
    enum { Scalar          = BaseTypeInfo::Scalar          }; ///< 1 if this type uses scalar values
    enum { Text            = BaseTypeInfo::Text            }; ///< 1 if this type uses text values
    enum { CopyOnWrite     = 1                             }; ///< 1 if this type uses copy-on-write

    enum { Size = BaseTypeInfo::Size }; ///< largest known fixed size for this type, as returned by size()
    static size_t size()
    {
        return BaseTypeInfo::size();
    }

    static size_t size(const DataType& data)
    {
        if (BaseTypeInfo::FixedSize)
            return data.size()*BaseTypeInfo::size();
        else
        {
            size_t n = data.size();
            size_t s = 0;
            for (size_t i=0; i<n; ++i)
                s+= BaseTypeInfo::size(data[index_type(i)]);
            return s;
        }
    }

    static void setSize(DataType& data, size_t size)
    {
        if (BaseTypeInfo::FixedSize)
            data.resize(size/BaseTypeInfo::size());
    }

    template <typename T>
    static void getValue(const DataType &data, size_t index, T& value)
    {
        if (BaseTypeInfo::FixedSize && BaseTypeInfo::size() == 1)
        {
            BaseTypeInfo::getValue(data[index_type(index)], 0, value);
        }
        else if (BaseTypeInfo::FixedSize)
        {
            BaseTypeInfo::getValue(data[index_type((index/BaseTypeInfo::size()))], (size_type)(index%BaseTypeInfo::size()), value);
        }
        else
        {
            size_t s = 0;
            for (size_t i=0; i<data.size(); ++i)
            {
                size_t n = BaseTypeInfo::size(data[index_type(i)]);
                if (index < s+n)
                {
                    BaseTypeInfo::getValue(data[index_type(i)], index-s, value);
                    break;
                }
                s += n;
            }
        }
    }

    template<typename T>
    static void setValue(DataType &data, size_t index, const T& value )
    {
        if (BaseTypeInfo::FixedSize && BaseTypeInfo::size() == 1)
        {
            BaseTypeInfo::setValue(data[index_type(index)], 0, value);
        }
        else if (BaseTypeInfo::FixedSize)
        {
            BaseTypeInfo::setValue(data[index_type((index/BaseTypeInfo::size()))], (size_type)(index%BaseTypeInfo::size()), value);
        }
        else
        {
            size_t s = 0;
            for (size_t i=0; i<data.size(); ++i)
            {
                size_t n = BaseTypeInfo::size(data[index_type(i)]);
                if (index < s+n)
                {
                    BaseTypeInfo::setValue(data[index_type(i)], index-s, value);
                    break;
                }
                s += n;
            }
        }
    }

    static void getValueString(const DataType &data, size_t index, std::string& value)
    {
        if (BaseTypeInfo::FixedSize && BaseTypeInfo::size() == 1)
        {
            BaseTypeInfo::getValueString(data[index_type(index)], 0, value);
        }
        else if (BaseTypeInfo::FixedSize)
        {
            BaseTypeInfo::getValueString(data[index_type((index/BaseTypeInfo::size()))], (size_type)(index%BaseTypeInfo::size()), value);
        }
        else
        {
            size_t s = 0;
            for (size_t i=0; i<data.size(); ++i)
            {
                size_t n = BaseTypeInfo::size(data[index_type(i)]);
                if (index < s+n)
                {
                    BaseTypeInfo::getValueString(data[index_type(i)], index-s, value);
                    break;
                }
                s += n;
            }
        }
    }

    static void setValueString(DataType &data, size_t index, const std::string& value )
    {
        if (BaseTypeInfo::FixedSize && BaseTypeInfo::size() == 1)
        {
            BaseTypeInfo::setValueString(data[index_type(index)], 0, value);
        }
        else if (BaseTypeInfo::FixedSize)
        {
            BaseTypeInfo::setValueString(data[index_type((index/BaseTypeInfo::size()))], (size_type)(index%BaseTypeInfo::size()), value);
        }
        else
        {
            size_t s = 0;
            for (size_t i=0; i<data.size(); ++i)
            {
                size_t n = BaseTypeInfo::size(data[index_type(i)]);
                if (index < s+n)
                {
                    BaseTypeInfo::setValueString(data[index_type(i)], index-s, value);
                    break;
                }
                s += n;
            }
        }
    }
};

} // namespace helper

namespace defaulttype
{

template <sofa::helper::integer_id_name Name, typename Index, Index DefaultId>
struct DataTypeInfo< sofa::helper::integer_id<Name, Index, DefaultId> > : public sofa::helper::IntegerIdTypeInfo<sofa::helper::integer_id<Name, Index, DefaultId> >
{
    static std::string name() { std::ostringstream o; o << "integer_id<" << Name() << "," << DataTypeName<Index>::name() << "," << DefaultId << ">"; return o.str(); }
};

template <class T, class TIndex, bool CheckIndices, class MemoryManager>
struct DataTypeInfo< sofa::helper::vector_id<T, TIndex, CheckIndices, MemoryManager> > : public sofa::helper::VectorIdTypeInfo< sofa::helper::vector_id<T, TIndex, CheckIndices, MemoryManager> >
{
    static std::string name() { std::ostringstream o; o << "vector_id<" << DataTypeName<T>::name() << "," << DataTypeName<TIndex>::name() << ">"; return o.str(); }
};

} // namespace defaulttype

} // namespace sofa

#endif
