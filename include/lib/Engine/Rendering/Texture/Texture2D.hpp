#pragma once
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
namespace EvoEngine
{
	enum class TextureColorType {
		Red = 1,
		RG = 2,
		RGB = 3,
		RGBA = 4
	};

	class Texture2D : public IAsset
	{
		friend class EditorLayer;
		friend class Resources;
		friend class Cubemap;
		friend class TextureStorage;
		friend class RenderLayer;
		std::shared_ptr<Image> m_image = {};
		std::shared_ptr<ImageView> m_imageView = {};
		std::shared_ptr<Sampler> m_sampler = {};
		ImTextureID m_imTextureId = VK_NULL_HANDLE;

		uint32_t m_textureStorageIndex = UINT32_MAX;
		void SetData(const void* data, const glm::uvec2& resolution);

	protected:
		bool SaveInternal(const std::filesystem::path& path) override;
		bool LoadInternal(const std::filesystem::path& path) override;
	public:
		
		[[nodiscard]] uint32_t GetTextureStorageIndex() const;
		~Texture2D() override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		[[nodiscard]] glm::vec2 GetResolution() const;
		void StoreToPng(
			const std::string& path,
			int resizeX = -1,
			int resizeY = -1,
			unsigned compressionLevel = 8) const;
		void StoreToJpg(const std::string& path, int resizeX = -1, int resizeY = -1, unsigned quality = 100) const;
		void StoreToHdr(const std::string& path, int resizeX = -1, int resizeY = -1,
			bool alphaChannel = false, unsigned quality = 100) const;
		ImTextureID GetImTextureId() const;

		[[nodiscard]] VkImageLayout GetLayout() const;
		[[nodiscard]] VkImage GetVkImage() const;
		[[nodiscard]] VkImageView GetVkImageView() const;
		[[nodiscard]] VkSampler GetVkSampler() const;
		[[nodiscard]] std::shared_ptr<Image> GetImage() const;
		void GetRgbaChannelData(std::vector<glm::vec4>& dst, int resizeX = -1, int resizeY = -1) const;
		void GetRgbChannelData(std::vector<glm::vec3>& dst, int resizeX = -1, int resizeY = -1) const;
		void GetRgChannelData(std::vector<glm::vec2>& dst, int resizeX = -1, int resizeY = -1) const;
		void GetRedChannelData(std::vector<float>& dst, int resizeX = -1, int resizeY = -1) const;

		void SetRgbaChannelData(const std::vector<glm::vec4>& src, const glm::uvec2& resolution);
		void SetRgbChannelData(const std::vector<glm::vec3>& src, const glm::uvec2& resolution);
		void SetRgChannelData(const std::vector<glm::vec2>& src, const glm::uvec2& resolution);
		void SetRedChannelData(const std::vector<float>& src, const glm::uvec2& resolution);
	};
}
