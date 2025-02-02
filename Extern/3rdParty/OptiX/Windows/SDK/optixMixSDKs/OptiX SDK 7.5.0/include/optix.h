
/* 
* SPDX-FileCopyrightText: Copyright (c) 2009 - 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved. 
* SPDX-License-Identifier: LicenseRef-NvidiaProprietary 
* 
* NVIDIA CORPORATION, its affiliates and licensors retain all intellectual 
* property and proprietary rights in and to this material, related 
* documentation and any modifications thereto. Any use, reproduction, 
* disclosure or distribution of this material and related documentation 
* without an express license agreement from NVIDIA CORPORATION or 
* its affiliates is strictly prohibited. 
*/
/// @file
/// @author NVIDIA Corporation
/// @brief  OptiX public API header
///
/// Includes the host api if compiling host code, includes the cuda api if compiling device code.
/// For the math library routines include optix_math.h

#ifndef __optix_optix_h__
#define __optix_optix_h__

/// The OptiX version.
///
/// - major =  OPTIX_VERSION/10000
/// - minor = (OPTIX_VERSION%10000)/100
/// - micro =  OPTIX_VERSION%100
#define OPTIX_VERSION 70500


#ifdef __CUDACC__
#include "optix_device.h"
#else
#include "optix_host.h"
#endif


#endif  // __optix_optix_h__
