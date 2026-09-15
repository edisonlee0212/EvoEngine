#pragma once

#include "EvoEngineEditorAPI.hpp"
#include "EvoEngine_SDK_PCH.hpp"

#define IMGUI_IMPL_VULKAN_USE_VOLK
#define IMGUI_DEFINE_MATH_OPERATORS  ///< Enables math operators in ImGui
#include <imgui.h>
#include <imgui_internal.h>
#include "Utilities/X11MacroCleanup.hpp"
#define IMGUI_IMPL_GLFW_DISABLE_X11
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_vulkan.h>
#include "Utilities/X11MacroCleanup.hpp"
// #include <imgui_stdlib.hpp>

#ifdef EVOENGINE_WINDOWS
#  include <backends/imgui_impl_win32.h>
#endif

#include <ImGuizmo.h>
#include "EditorWidgets.hpp"
#include "ImGuiFileDialog.hpp"
#include "ImGuiFileDialogConfig.hpp"
#include "imnodes_internal.hpp"
