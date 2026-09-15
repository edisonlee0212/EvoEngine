#include <unordered_map>
#include "AssetThumbnailProvider.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "EditorTextureRegistry.hpp"
#include "FileManager.hpp"
#include "InspectorRegistry.hpp"
#include "NodeGraphEditor.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "Texture2D.hpp"

using namespace evo_engine;

namespace {
struct InspectorThumbnailCache {
  uint64_t asset_handle = 0;
  uint32_t asset_version = 0;
  glm::vec2 subject_rotation = glm::vec2(0.0f);
  float camera_zoom = 1.0f;
  std::shared_ptr<Texture2D> thumbnail;
};

struct InspectorPreviewInteraction {
  uint64_t asset_handle = 0;
  bool interaction_mode = false;
  glm::vec2 subject_rotation = glm::vec2(0.0f);
  float camera_zoom = 1.0f;
};

struct InspectorPreviewState {
  std::unordered_map<uint64_t, InspectorThumbnailCache> thumbnails;
  std::unordered_map<uint64_t, InspectorPreviewInteraction> interactions;
};
auto& PreviewStates() {
  static std::unordered_map<Application*, InspectorPreviewState> states;
  return states;
}
InspectorPreviewState& PreviewState() {
  auto* application = &ApplicationContext::Get();
  auto [it, inserted] = PreviewStates().try_emplace(application);
  if (inserted)
    static_cast<void>(application->RegisterCleanupFunction([application] {
      PreviewStates().erase(application);
    }));
  return it->second;
}

constexpr float kInspectorPreviewRotationSensitivity = 0.01f;
constexpr float kInspectorPreviewMinPitch = -1.4f;
constexpr float kInspectorPreviewMaxPitch = 1.4f;
constexpr float kInspectorPreviewMinZoom = 0.4f;
constexpr float kInspectorPreviewMaxZoom = 3.0f;
bool IsInspectorPreviewInteractive(const std::shared_ptr<IAsset>& asset) {
  if (!asset) {
    return false;
  }
  const auto& type_name = asset->GetTypeName();
  return type_name == "Material" || type_name == "Mesh";
}

InspectorPreviewInteraction& GetInspectorPreviewInteraction(const std::shared_ptr<IAsset>& asset) {
  const uint64_t asset_handle = asset ? asset->GetHandle().GetValue() : 0;
  auto& preview_interaction = PreviewState().interactions[asset_handle];
  preview_interaction.asset_handle = asset_handle;
  return preview_interaction;
}

OffscreenPreviewSettings CreateInspectorPreviewSettings(const std::shared_ptr<IAsset>& asset) {
  OffscreenPreviewSettings settings;
  if (IsInspectorPreviewInteractive(asset)) {
    const auto& preview_interaction = GetInspectorPreviewInteraction(asset);
    settings.subject_rotation = preview_interaction.subject_rotation;
    settings.camera_zoom = preview_interaction.camera_zoom;
  }
  return settings;
}

bool InspectorThumbnailCacheMatches(const uint64_t asset_handle, const uint32_t asset_version,
                                    const OffscreenPreviewSettings& settings) {
  const auto search = PreviewState().thumbnails.find(asset_handle);
  return search != PreviewState().thumbnails.end() && search->second.asset_version == asset_version &&
         search->second.subject_rotation == settings.subject_rotation &&
         search->second.camera_zoom == settings.camera_zoom;
}

std::shared_ptr<Texture2D> GetInspectorThumbnail(const std::shared_ptr<IAsset>& asset,
                                                 const OffscreenPreviewSettings& settings) {
  if (!asset || !AssetThumbnailProvider::SupportsGeneratedThumbnail(asset->GetTypeName())) {
    return {};
  }

  const auto asset_handle = asset->GetHandle().GetValue();
  const auto asset_version = asset->GetVersion();
  if (InspectorThumbnailCacheMatches(asset_handle, asset_version, settings)) {
    return PreviewState().thumbnails[asset_handle].thumbnail;
  }

  auto& cache = PreviewState().thumbnails[asset_handle];
  cache.asset_handle = asset_handle;
  cache.asset_version = asset_version;
  cache.subject_rotation = settings.subject_rotation;
  cache.camera_zoom = settings.camera_zoom;
  cache.thumbnail = AssetThumbnailProvider::GenerateThumbnail(asset, settings);
  return cache.thumbnail;
}

void InvalidateInspectorThumbnailCache(const std::shared_ptr<IAsset>& asset) {
  if (!asset) {
    return;
  }

  PreviewState().thumbnails.erase(asset->GetHandle().GetValue());
}

void ResetInspectorPreviewInteraction(const std::shared_ptr<IAsset>& asset,
                                      InspectorPreviewInteraction& preview_interaction) {
  const bool changed = preview_interaction.interaction_mode ||
                       preview_interaction.subject_rotation != glm::vec2(0.0f) ||
                       preview_interaction.camera_zoom != 1.0f;
  preview_interaction.interaction_mode = false;
  preview_interaction.subject_rotation = glm::vec2(0.0f);
  preview_interaction.camera_zoom = 1.0f;
  if (changed) {
    InvalidateInspectorThumbnailCache(asset);
  }
}

void DrawInspectorThumbnail(const std::shared_ptr<IAsset>& asset) {
  if (!IsInspectorPreviewInteractive(asset)) {
    return;
  }

  auto& preview_interaction = GetInspectorPreviewInteraction(asset);
  const auto settings = CreateInspectorPreviewSettings(asset);
  const auto thumbnail = GetInspectorThumbnail(asset, settings);
  if (!thumbnail) {
    return;
  }

  const float available_width = ImGui::GetContentRegionAvail().x;
  const float preview_extent = available_width;
  if (preview_extent <= 0.0f) {
    return;
  }

  ImVec2 image_size(preview_extent, preview_extent);
  const glm::vec2 texture_resolution = thumbnail->GetResolution();
  if (texture_resolution.x > 0.0f && texture_resolution.y > 0.0f) {
    if (texture_resolution.x > texture_resolution.y) {
      image_size.y *= texture_resolution.y / texture_resolution.x;
    } else {
      image_size.x *= texture_resolution.x / texture_resolution.y;
    }
  }

  ImGui::Separator();
  ImGui::TextUnformatted("Preview");
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (available_width - image_size.x) * 0.5f));
  ImGui::InvisibleButton("##InspectorAssetPreview", image_size);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  const ImVec2 image_min = ImGui::GetItemRectMin();
  const ImVec2 image_max = ImGui::GetItemRectMax();
  auto* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddImage(EditorTextureRegistry::GetTextureId(*thumbnail), image_min, image_max, ImVec2(0, 1),
                      ImVec2(1, 0));

  if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
    preview_interaction.interaction_mode = !preview_interaction.interaction_mode;
  }

  if (preview_interaction.interaction_mode) {
    if (!hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      ResetInspectorPreviewInteraction(asset, preview_interaction);
      return;
    }
    draw_list->AddRect(image_min, image_max, ImGui::GetColorU32(ImGuiCol_NavHighlight), 0.0f, 0, 2.0f);
    if (hovered || active) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    }

    bool interaction_changed = false;
    const auto& io = ImGui::GetIO();
    if ((hovered || active) && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)) {
      const ImVec2 mouse_delta = io.MouseDelta;
      if (mouse_delta.x != 0.0f || mouse_delta.y != 0.0f) {
        preview_interaction.subject_rotation.x += mouse_delta.x * kInspectorPreviewRotationSensitivity;
        preview_interaction.subject_rotation.y =
            std::clamp(preview_interaction.subject_rotation.y + mouse_delta.y * kInspectorPreviewRotationSensitivity,
                       kInspectorPreviewMinPitch, kInspectorPreviewMaxPitch);
        interaction_changed = true;
      }
    }
    if (hovered && io.MouseWheel != 0.0f) {
      preview_interaction.camera_zoom = std::clamp(preview_interaction.camera_zoom * std::pow(1.12f, io.MouseWheel),
                                                   kInspectorPreviewMinZoom, kInspectorPreviewMaxZoom);
      interaction_changed = true;
    }
    if (interaction_changed) {
      InvalidateInspectorThumbnailCache(asset);
    }
  }
}
}  // namespace

void EditorLayer::ClearInspectorPreviewState() {
  NodeGraphEditor::Clear();
  AssetThumbnailProvider::ClearFileThumbnails();
  OffscreenPreviewRenderer::Reset();
  const auto it = PreviewStates().find(ApplicationContext::TryGet());
  if (it != PreviewStates().end())
    it->second = {};
}

void EditorLayer::DrawAssetInspectorContent(const std::shared_ptr<EditorLayer>& editor_layer,
                                            const std::shared_ptr<IAsset>& asset) {
  if (asset) {
    bool asset_changed = false;
    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_Header));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_HeaderActive));
    ImGui::Button(asset->GetTitle().c_str());
    ImGui::PopStyleColor(3);
    editor_layer->DraggableAsset(asset);
    ImGui::SameLine();
    ImGui::Text("Type:");
    ImGui::SameLine();
    ImGui::Text(asset->GetTypeName().c_str());
    if (!asset->IsTemporary()) {
      if (ImGui::Button("Save")) {
        asset->Save();
      }
      ImGui::SameLine();
      if (ImGui::Button("Reload")) {
        asset_changed = asset->Load();
      }
    }
    ImGui::SameLine();
    EditorFileDialogs::SaveFile(
        "Export...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
        [&](const std::filesystem::path& path) {
          asset->Export(path);
        },
        false);
    ImGui::SameLine();
    EditorFileDialogs::OpenFile(
        "Import...", asset->GetTypeName(), Serialization::PeekAssetExtensions(asset->GetTypeName()),
        [&](const std::filesystem::path& path) {
          asset_changed = asset->Import(path);
        },
        false);

    if (asset_changed) {
      editor_layer->ClearEnvironmentalLightingGizmoTarget(asset->GetHandle());
      InvalidateInspectorThumbnailCache(asset);
      if (const auto file = asset->GetFileRecord().lock()) {
        file->NotifyContentChanged();
      }
    }
    DrawInspectorThumbnail(asset);

    ImGui::Separator();
    InspectorContext context;
    context.editor_layer = editor_layer;
    if (InspectorRegistry::GetInstance().Inspect(context, *asset)) {
      asset->SetUnsaved();
      InvalidateInspectorThumbnailCache(asset);
      if (const auto file = asset->GetFileRecord().lock()) {
        file->NotifyContentChanged();
      }
    }
  } else {
    ImGui::Text("None");
  }
}
