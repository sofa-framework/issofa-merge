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

#include "Binding_BaseLoader.h"
#include "Binding_BaseObject.h"

using namespace sofa::core::loader;
using namespace sofa::core;
using namespace sofa::core::objectmodel;



extern "C" PyObject * BaseLoader_load(PyObject *self, PyObject * /*args*/)
{
    BaseLoader* obj=((PySPtr<Base>*)self)->object->toBaseLoader();
    bool result = obj->load();
    return PyBool_FromLong(result);
}

extern "C" PyObject * BaseLoader_canLoad(PyObject *self, PyObject * /*args*/)
{
    BaseLoader* obj=((PySPtr<Base>*)self)->object->toBaseLoader();
    bool result = obj->canLoad();
    return PyBool_FromLong(result);
}

extern "C" PyObject * BaseLoader_setFilename(PyObject *self, PyObject * args)
{
    BaseLoader* obj=((PySPtr<Base>*)self)->object->toBaseLoader();
    char *filename;
    if (!PyArg_ParseTuple(args, "s",&filename))
    {
        PyErr_BadArgument();
        Py_RETURN_NONE;
    }
    obj->setFilename(filename);
    Py_RETURN_NONE;
}

extern "C" PyObject * BaseLoader_getFilename(PyObject *self, PyObject * /*args*/)
{
    BaseLoader* obj=((PySPtr<Base>*)self)->object->toBaseLoader();
    std::string filename = obj->getFilename();
    return PyString_FromString(filename.c_str());
}



SP_CLASS_METHODS_BEGIN(BaseLoader)
SP_CLASS_METHOD(BaseLoader,load)
SP_CLASS_METHOD(BaseLoader,canLoad)
SP_CLASS_METHOD(BaseLoader,setFilename)
SP_CLASS_METHOD(BaseLoader,getFilename)
//SP_CLASS_METHOD(BaseLoader,skipToEOL)
//SP_CLASS_METHOD(BaseLoader,readLine)
SP_CLASS_METHODS_END


SP_CLASS_TYPE_SPTR(BaseLoader,BaseLoader,BaseObject)

