#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include <IPrivateComponent.hpp>
#include <RenderTexture.hpp>


#include "Graphics.hpp"
#include "CUDAModule.hpp"

namespace evo_engine {
    class RayTracerCamera : public IPrivateComponent {
        friend class RayTracerLayer;
        friend class RayTracer;
        CameraProperties camera_properties_;
        bool rendered_ = false;
        bool main_camera_ = false;
        AssetRef skybox_;
    public:
        void SetSkybox(const std::shared_ptr<Cubemap>& cubemap);

        void SetMainCamera(bool value);
        bool allow_auto_resize = true;
        std::shared_ptr<RenderTexture> render_texture;
        RayProperties ray_properties;
        glm::uvec2 frame_size;
        void Ready(const glm::vec3& position, const glm::quat& rotation);
        bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
        void SetFov(float value);
        void SetAperture(float value);
        void SetFocalLength(float value);
        void SetMaxDistance(float value);
        void SetOutputType(OutputType value);
        void SetAccumulate(bool value);
        void SetGamma(float value);
        [[nodiscard]] glm::mat4 GetProjection() const;
        void SetDenoiserStrength(float value);
        void OnCreate() override;
        void OnDestroy() override;
        void Serialize(YAML::Emitter &out) const override;
        void Deserialize(const YAML::Node &in) override;
        RayTracerCamera& operator=(const RayTracerCamera& source);
        void Render();
        void Render(const RayProperties& ray_properties);
        void Render(const RayProperties& ray_properties, const EnvironmentProperties& environment_properties);
    };
}