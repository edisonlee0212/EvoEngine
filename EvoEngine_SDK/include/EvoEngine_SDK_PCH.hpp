
/**
 * @file MainHeader.hpp
 * @brief This file includes all the necessary headers and configurations
 *        used throughout the project. It handles platform-specific definitions
 *        and includes libraries like Vulkan, ImGui, Assimp, stb_image, YAML, etc.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdarg>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "Math.hpp"

// Platform-specific Definitions
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  define EVOENGINE_WINDOWS  ///< Indicates Windows platform
#elif __APPLE__
#  define EVOENGINE_MACOS  ///< Indicates macOS platform
#else
#  define EVOENGINE_LINUX  ///< Indicates Linux platform
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#  include <cstdint>
#else
#  include <inttypes.h>
#endif

/// OpenGL and Vulkan Libraries

#include "volk.h"

#define VMA_STATIC_VULKAN_FUNCTIONS 0   ///< Disables static Vulkan functions for VMA
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 1  ///< Enables dynamic Vulkan functions for VMA
#include "vk_mem_alloc.h"

#define GLFW_INCLUDE_VULKAN  ///< Enables Vulkan support in GLFW
#define GLFW_INCLUDE_NONE    ///< Disables default OpenGL bindings in GLFW
#include "GLFW/glfw3.h"

#define IMGUI_DEFINE_MATH_OPERATORS  ///< Enables math operators in ImGui
#include <imgui.h>
#include <imgui_internal.h>

// #include <imgui_stdlib.hpp>

#ifdef EVOENGINE_WINDOWS
#  define GLFW_EXPOSE_NATIVE_WIN32  ///< Exposes native Win32 context for GLFW
#  include "GLFW/glfw3native.h"
#  define STBI_MSC_SECURE_CRT  ///< Configures CRT secure functions for stb_image on Windows
// define something for Windows (32-bit and 64-bit, this part is common)
#  include <backends/imgui_impl_glfw.h>
#  include <backends/imgui_impl_vulkan.h>
#  include <backends/imgui_impl_win32.h>
#else
// Linux specific includes
#  include <backends/imgui_impl_vulkan.h>
#  include <imgui_impl_glfw.h>
#endif

#include <ImGuizmo.h>

/// Assimp Libraries and Configurations
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>

// stb_image Libraries
#include <stb_image.h>
#include <stb_image_resize2.h>
#include <stb_image_write.h>

// YAML Parsing Library
#include <yaml-cpp/yaml.h>

// Debug Configuration Macros
#ifdef _DEBUG
#  undef _DEBUG
#  define DEBUG_WAS_DEFINED  ///< Sets a flag indicating debug was originally defined
#endif
#ifndef NDEBUG
#  define NDEBUG                  ///< Indicates non-debug configuration
#  define NDEBUG_WAS_NOT_DEFINED  ///< Sets a flag indicating NDEBUG was not originally defined
#endif

#ifdef DEBUG_WAS_DEFINED
#  undef DEBUG_WAS_DEFINED
#  define _DEBUG  ///< Restores the original debug macro
#endif

#ifdef NDEBUG_WAS_NOT_DEFINED
#  undef NDEBUG_WAS_NOT_DEFINED
#  undef NDEBUG  ///< Clears the NDEBUG macro
#endif

#ifdef EVOENGINE_WINDOWS
#  include <Windows.h>
#endif

#include "ImGuiFileDialog.hpp"
#include "ImGuiFileDialogConfig.hpp"
#include "imnodes_internal.hpp"
