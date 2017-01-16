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
#include "CudaTypes.h"
#include <sofa/core/ObjectFactory.h>
#include <SofaGeneralEngine/IndexValueMapper.inl>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace engine
{

template class IndexValueMapper<gpu::cuda::CudaVector<defaulttype::Vec3f> >;
template class IndexValueMapper<gpu::cuda::CudaVector<gpu::cuda::Vec3f1> >;
#ifdef SOFA_GPU_CUDA_DOUBLE
template class IndexValueMapper<gpu::cuda::CudaVector<defaulttype::Vec3d> >;
template class IndexValueMapper<gpu::cuda::CudaVector<gpu::cuda::Vec3d1> >;
#endif // SOFA_GPU_CUDA_DOUBLE

} // namespace engine

} // namespace component

namespace gpu
{

namespace cuda
{

SOFA_DECL_CLASS(CudaIndexValueMapper)

int IndexValueMapperClass = core::RegisterObject("Supports GPU-side computations using CUDA")
        .add< component::engine::IndexValueMapper<gpu::cuda::CudaVector<defaulttype::Vec3f> > >()
        .add< component::engine::IndexValueMapper<gpu::cuda::CudaVector<gpu::cuda::Vec3f1> > >()
#ifdef SOFA_GPU_CUDA_DOUBLE
        .add< component::engine::IndexValueMapper<gpu::cuda::CudaVector<defaulttype::Vec3d> > >()
        .add< component::engine::IndexValueMapper<gpu::cuda::CudaVector<gpu::cuda::Vec3d1> > >()
#endif // SOFA_GPU_CUDA_DOUBLE
        ;

} // namespace cuda

} // namespace gpu

} // namespace sofa
