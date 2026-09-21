#include "GuiTextureRegistry.hpp"
#include <imgui_impl_vulkan.h>
#include "Application.hpp"
#include "Platform.hpp"
#include "RenderTexture.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"

using namespace evo_engine;

namespace {
struct GuiTexture {
  std::shared_ptr<Image> image;
  std::shared_ptr<ImageView> view;
  std::shared_ptr<Sampler> sampler;
  VkImageLayout layout;
  VkDescriptorSet descriptor;
  GuiTexture(std::shared_ptr<Image> source, std::shared_ptr<ImageView> source_view,
             std::shared_ptr<Sampler> source_sampler)
      : image(std::move(source)),
        view(std::move(source_view)),
        sampler(std::move(source_sampler)),
        layout(image->GetLayout()),
        descriptor(ImGui_ImplVulkan_AddTexture(sampler->GetVkSampler(), view->GetVkImageView(), layout)) {
  }
  GuiTexture(const GuiTexture&) = delete;
  GuiTexture& operator=(const GuiTexture&) = delete;
  ~GuiTexture() {
    ImGui_ImplVulkan_RemoveTexture(descriptor);
  }
};
struct TextureFrame {
  uint32_t number;
  std::shared_ptr<FrameSubmissionState> submission;
  std::vector<std::shared_ptr<GuiTexture>> textures;
};
struct TextureCache {
  std::vector<std::weak_ptr<GuiTexture>> textures;
  std::vector<TextureFrame> frames;
};
auto& Caches() {
  static std::unordered_map<Application*, std::unique_ptr<TextureCache>> caches;
  return caches;
}
TextureCache& GetCache() {
  auto* application = &ApplicationContext::Get();
  auto& cache = Caches()[application];
  if (!cache) {
    cache = std::make_unique<TextureCache>();
    static_cast<void>(application->RegisterCleanupFunction([application] {
      Caches().erase(application);
    }));
  }
  return *cache;
}
}  // namespace

ImTextureID GuiTextureRegistry::GetTextureId(const std::shared_ptr<Image>& image,
                                             const std::shared_ptr<ImageView>& view,
                                             const std::shared_ptr<Sampler>& sampler) {
  if (!ImGui::GetCurrentContext() || !image || !view || !sampler)
    return 0;
  auto& cache = GetCache();
  std::shared_ptr<GuiTexture> texture;
  for (const auto& candidate : cache.textures) {
    if (auto existing = candidate.lock(); existing && existing->image == image && existing->view == view &&
                                          existing->sampler == sampler && existing->layout == image->GetLayout()) {
      texture = std::move(existing);
      break;
    }
  }
  if (!texture) {
    texture = std::make_shared<GuiTexture>(image, view, sampler);
    cache.textures.emplace_back(texture);
  }
  const auto frame = Platform::GetFrameCount();
  if (cache.frames.empty() || cache.frames.back().number != frame ||
      cache.frames.back().submission->status != FrameSubmissionState::Status::Pending) {
    cache.frames.push_back({frame, Platform::TrackCurrentFrameSubmission(), {}});
  }
  auto& textures = cache.frames.back().textures;
  if (std::find(textures.begin(), textures.end(), texture) == textures.end())
    textures.push_back(texture);
  return reinterpret_cast<ImTextureID>(texture->descriptor);
}

ImTextureID GuiTextureRegistry::GetTextureId(const SampledImageResources& resources) {
  return GetTextureId(resources.image, resources.image_view, resources.sampler);
}

ImTextureID GuiTextureRegistry::GetTextureId(const Texture2D& texture) {
  const auto& storage = texture.PeekTexture2DStorage();
  return GetTextureId(storage.image, storage.image_view, storage.sampler);
}

ImTextureID GuiTextureRegistry::GetColorTextureId(RenderTexture& texture, uint32_t mip) {
  if (!texture.GetColorImage())
    return 0;
  return GetTextureId(texture.GetColorImage(), texture.GetColorImageView(mip), texture.GetColorSampler());
}

ImTextureID GuiTextureRegistry::GetDepthTextureId(RenderTexture& texture, uint32_t mip) {
  if (!texture.GetDepthImage())
    return 0;
  return GetTextureId(texture.GetDepthImage(), texture.GetDepthImageView(mip), texture.GetDepthSampler());
}

void GuiTextureRegistry::CollectGarbage() {
  const auto found = Caches().find(ApplicationContext::TryGet());
  if (found == Caches().end())
    return;
  auto& cache = *found->second;
  cache.frames.erase(std::remove_if(cache.frames.begin(), cache.frames.end(),
                                    [](const auto& frame) {
                                      return frame.submission->status != FrameSubmissionState::Status::Pending;
                                    }),
                     cache.frames.end());
  cache.textures.erase(std::remove_if(cache.textures.begin(), cache.textures.end(),
                                      [](const auto& texture) {
                                        return texture.expired();
                                      }),
                       cache.textures.end());
}
