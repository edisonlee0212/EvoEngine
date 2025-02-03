/* 
* SPDX-FileCopyrightText: Copyright (c) 2019 - 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved. 
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

#ifndef __optix_optix_function_table_definition_h__
#define __optix_optix_function_table_definition_h__

#include "optix_function_table.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup optix_function_table
@{
*/

/// If the stubs in optix_stubs.h are used, then the function table needs to be defined in exactly
/// one translation unit. This can be achieved by including this header file in that translation
/// unit.
OptixFunctionTable g_optixFunctionTable;

/*@}*/  // end group optix_function_table

#ifdef __cplusplus
}
#endif

#endif  // __optix_optix_function_table_definition_h__
