#pragma once

#include "EvoEngineBuildIdentity.hpp"
#include "EvoEnginePackageBuildIdentity.hpp"

#define EVOENGINE_PACKAGE_BUILD_IDENTITY                                                               \
  evo_engine::NativeBuildIdentity {                                                                    \
    EVOENGINE_SDK_SOURCE_ID, EVOENGINE_NATIVE_COMPILER_ID, EVOENGINE_NATIVE_COMPILER_VERSION,          \
        EVOENGINE_NATIVE_BUILD_CONFIGURATION, EVOENGINE_NATIVE_PLATFORM, EVOENGINE_NATIVE_ARCHITECTURE \
  }

#include "Application.hpp"
#include "AssetManager.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Utilities.hpp"
#include "WindowLayer.hpp"
