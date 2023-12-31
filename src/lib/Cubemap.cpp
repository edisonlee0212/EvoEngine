#include "Cubemap.hpp"

#include "Application.hpp"
#include "Console.hpp"
#include "Graphics.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "TextureStorage.hpp"
using namespace EvoEngine;
#include "EditorLayer.hpp"
struct EquirectangularToCubemapConstant
{
	glm::mat4 m_projectionView = {};
	float m_presetValue = 0;
};

Cubemap::~Cubemap()
{
	const auto self = GetSelf();
	if(self) TextureStorage::UnRegisterCubemap(std::dynamic_pointer_cast<Cubemap>(self));
}

void Cubemap::Initialize(uint32_t resolution, uint32_t mipLevels)
{
	m_sampler.reset();
	m_imageView.reset();
	m_image.reset();


	m_faceViews.clear();

	VkImageCreateInfo imageInfo{};
	imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;
	imageInfo.extent.width = resolution;
	imageInfo.extent.height = resolution;
	imageInfo.extent.depth = 1;
	imageInfo.mipLevels = mipLevels;
	imageInfo.arrayLayers = 6;
	imageInfo.format = Graphics::Constants::TEXTURE_2D;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	imageInfo.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	m_image = std::make_shared<Image>(imageInfo);

	VkImageViewCreateInfo viewInfo{};
	viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	viewInfo.image = m_image->GetVkImage();
	viewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
	viewInfo.format = Graphics::Constants::TEXTURE_2D;
	viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	viewInfo.subresourceRange.baseMipLevel = 0;
	viewInfo.subresourceRange.levelCount = mipLevels;
	viewInfo.subresourceRange.baseArrayLayer = 0;
	viewInfo.subresourceRange.layerCount = 6;

	m_imageView = std::make_shared<ImageView>(viewInfo);


	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerInfo.magFilter = VK_FILTER_LINEAR;
	samplerInfo.minFilter = VK_FILTER_LINEAR;
	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerInfo.anisotropyEnable = VK_TRUE;
	samplerInfo.maxAnisotropy = Graphics::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;
	samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
	samplerInfo.unnormalizedCoordinates = VK_FALSE;
	samplerInfo.compareEnable = VK_FALSE;
	samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	if (mipLevels > 1) {
		samplerInfo.minLod = 0;
		samplerInfo.maxLod = static_cast<float>(mipLevels);
	}
	m_sampler = std::make_shared<Sampler>(samplerInfo);

#pragma endregion
	Graphics::ImmediateSubmit([&](VkCommandBuffer commandBuffer) {
		m_image->TransitImageLayout(commandBuffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		}
	);

	for (int i = 0; i < 6; i++)
	{
		VkImageViewCreateInfo viewInfo{};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.image = m_image->GetVkImage();
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = Graphics::Constants::TEXTURE_2D;
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewInfo.subresourceRange.baseMipLevel = 0;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = i;
		viewInfo.subresourceRange.layerCount = 1;

		m_faceViews.emplace_back(std::make_shared<ImageView>(viewInfo));
	}

	m_imTextureIds.resize(6);
	for (int i = 0; i < 6; i++)
	{
		EditorLayer::UpdateTextureId(m_imTextureIds[i], m_sampler->GetVkSampler(), m_faceViews[i]->GetVkImageView(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
	}
	TextureStorage::RegisterCubemap(std::dynamic_pointer_cast<Cubemap>(GetSelf()));
}

uint32_t Cubemap::GetTextureStorageIndex() const
{
	return m_textureStorageIndex;
}

void Cubemap::ConvertFromEquirectangularTexture(const std::shared_ptr<Texture2D>& targetTexture)
{
	if (!m_image) Initialize(1024);

	if (!targetTexture->m_image) {
		EVOENGINE_ERROR("Target texture doesn't contain any content!");
		return;
	}
#pragma region Create image

#pragma region Depth
	VkImageCreateInfo depthImageInfo{};
	depthImageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	depthImageInfo.imageType = VK_IMAGE_TYPE_2D;
	depthImageInfo.extent.width = m_image->GetExtent().width;
	depthImageInfo.extent.height = m_image->GetExtent().height;
	depthImageInfo.extent.depth = 1;
	depthImageInfo.mipLevels = 1;
	depthImageInfo.arrayLayers = 1;
	depthImageInfo.format = Graphics::Constants::SHADOW_MAP;
	depthImageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	depthImageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	depthImageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	depthImageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	depthImageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	auto depthImage = std::make_shared<Image>(depthImageInfo);
	Graphics::ImmediateSubmit([&](VkCommandBuffer commandBuffer)
		{
			depthImage->TransitImageLayout(commandBuffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
		});


	VkImageViewCreateInfo depthViewInfo{};
	depthViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	depthViewInfo.image = depthImage->GetVkImage();
	depthViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
	depthViewInfo.format = Graphics::Constants::SHADOW_MAP;
	depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	depthViewInfo.subresourceRange.baseMipLevel = 0;
	depthViewInfo.subresourceRange.levelCount = 1;
	depthViewInfo.subresourceRange.baseArrayLayer = 0;
	depthViewInfo.subresourceRange.layerCount = 1;
	auto depthImageView = std::make_shared<ImageView>(depthViewInfo);
#pragma endregion

	std::unique_ptr<DescriptorSet> tempSet = std::make_unique<DescriptorSet>(Graphics::GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));
	VkDescriptorImageInfo descriptorImageInfo{};
	descriptorImageInfo.imageView = targetTexture->GetVkImageView();
	descriptorImageInfo.imageLayout = targetTexture->GetLayout();
	descriptorImageInfo.sampler = targetTexture->GetVkSampler();

	tempSet->UpdateImageDescriptorBinding(0, descriptorImageInfo);


	glm::mat4 captureProjection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
	glm::mat4 captureViews[] = {
		glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
		glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
		glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
		glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)),
		glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
		glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f)) };

	auto equirectangularToCubemap = Graphics::GetGraphicsPipeline("EQUIRECTANGULAR_TO_CUBEMAP");
	Graphics::ImmediateSubmit([&](VkCommandBuffer commandBuffer) {
		m_image->TransitImageLayout(commandBuffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
#pragma region Viewport and scissor
		VkRect2D renderArea;
		renderArea.offset = { 0, 0 };
		renderArea.extent.width = m_image->GetExtent().width;
		renderArea.extent.height = m_image->GetExtent().height;
		VkViewport viewport;
		viewport.x = 0.0f;
		viewport.y = 0.0f;
		viewport.width = m_image->GetExtent().width;
		viewport.height = m_image->GetExtent().height;
		viewport.minDepth = 0.0f;
		viewport.maxDepth = 1.0f;

		VkRect2D scissor;
		scissor.offset = { 0, 0 };
		scissor.extent.width = m_image->GetExtent().width;
		scissor.extent.height = m_image->GetExtent().height;
		equirectangularToCubemap->m_states.m_viewPort = viewport;
		equirectangularToCubemap->m_states.m_scissor = scissor;
#pragma endregion
		for (int i = 0; i < 6; i++) {
#pragma region Lighting pass
				VkRenderingAttachmentInfo attachment{};
				attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

				attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
				attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

				attachment.clearValue = { 0, 0, 0, 1 };
				attachment.imageView = m_faceViews[i]->GetVkImageView();

				VkRenderingAttachmentInfo depthAttachment{};
				depthAttachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

				depthAttachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
				depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

				depthAttachment.clearValue.depthStencil = { 1, 0 };
				depthAttachment.imageView = depthImageView->GetVkImageView();

				VkRenderingInfo renderInfo{};
				renderInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
				renderInfo.renderArea = renderArea;
				renderInfo.layerCount = 1;
				renderInfo.colorAttachmentCount = 1;
				renderInfo.pColorAttachments = &attachment;
				renderInfo.pDepthAttachment = &depthAttachment;
				equirectangularToCubemap->m_states.m_cullMode = VK_CULL_MODE_NONE;
				equirectangularToCubemap->m_states.m_colorBlendAttachmentStates.clear();
				equirectangularToCubemap->m_states.m_colorBlendAttachmentStates.resize(1);
				for (auto& i : equirectangularToCubemap->m_states.m_colorBlendAttachmentStates)
				{
					i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
					i.blendEnable = VK_FALSE;
				}
				vkCmdBeginRendering(commandBuffer, &renderInfo);
				equirectangularToCubemap->Bind(commandBuffer);
				equirectangularToCubemap->BindDescriptorSet(commandBuffer, 0, tempSet->GetVkDescriptorSet());
				const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_RENDERING_CUBE");
				GeometryStorage::BindVertices(commandBuffer);
				EquirectangularToCubemapConstant constant{};
				constant.m_projectionView = captureProjection * captureViews[i];
				equirectangularToCubemap->PushConstant(commandBuffer, 0, constant);
				mesh->DrawIndexed(commandBuffer, equirectangularToCubemap->m_states, 1);
				vkCmdEndRendering(commandBuffer);
#pragma endregion
			
			Graphics::EverythingBarrier(commandBuffer);
		}
		m_image->TransitImageLayout(commandBuffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		}
	);

}
void Cubemap::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if (!m_imTextureIds.empty()) {
		static float debugSacle = 0.25f;
		ImGui::DragFloat("Scale", &debugSacle, 0.01f, 0.1f, 1.0f);
		debugSacle = glm::clamp(debugSacle, 0.1f, 1.0f);
		for (int i = 0; i < 6; i++) {
			ImGui::Image(m_imTextureIds[i],
				ImVec2(m_image->GetExtent().width * debugSacle, m_image->GetExtent().height * debugSacle),
				ImVec2(0, 1),
				ImVec2(1, 0));
		}
	}
}

const std::shared_ptr<Image>& Cubemap::GetImage() const
{
	return m_image;
}

const std::shared_ptr<ImageView>& Cubemap::GetImageView() const
{
	return m_imageView;

}

const std::shared_ptr<Sampler>& Cubemap::GetSampler() const
{
	return m_sampler;
}

const std::vector<std::shared_ptr<ImageView>>& Cubemap::GetFaceViews() const
{
	return m_faceViews;
}
