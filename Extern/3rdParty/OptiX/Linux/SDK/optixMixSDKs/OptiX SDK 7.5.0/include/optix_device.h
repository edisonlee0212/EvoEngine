
/* 
* SPDX-FileCopyrightText: Copyright (c) 2010 - 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved. 
* SPDX-License-Identifier: LicenseRef-NvidiaProprietary 
* 
* NVIDIA CORPORATION, its affiliates and licensors retain all intellectual 
* property and proprietary rights in and to this material, related 
* documentation and any modifications thereto. Any use, reproduction, 
* disclosure or distribution of this material and related documentation 
* without an express license agreement from NVIDIA CORPORATION or 
* its affiliates is strictly prohibited. 
*/
 /**
 * @file   optix_device.h
 * @author NVIDIA Corporation
 * @brief  OptiX public API
 *
 * OptiX public API Reference - Host/Device side
 */

/******************************************************************************\
 * optix_cuda.h
 *
 * This file provides the nvcc interface for generating PTX that the OptiX is
 * capable of parsing and weaving into the final kernel.  This is included by
 * optix.h automatically if compiling device code.  It can be included explicitly
 * in host code if desired.
 *
\******************************************************************************/
#if !defined(__OPTIX_INCLUDE_INTERNAL_HEADERS__)
#  define __OPTIX_INCLUDE_INTERNAL_HEADERS__
#  define __UNDEF_OPTIX_INCLUDE_INTERNAL_HEADERS_OPTIX_DEVICE_H__
#endif
#include "optix_7_device.h"
#if defined( __UNDEF_OPTIX_INCLUDE_INTERNAL_HEADERS_OPTIX_DEVICE_H__ )
#  undef __OPTIX_INCLUDE_INTERNAL_HEADERS__
#  undef __UNDEF_OPTIX_INCLUDE_INTERNAL_HEADERS_OPTIX_DEVICE_H__
#endif
