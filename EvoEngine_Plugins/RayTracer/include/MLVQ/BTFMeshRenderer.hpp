#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "IPrivateComponent.hpp"

namespace evo_engine {
    class BTFMeshRenderer : public IPrivateComponent {
    public:
        AssetRef mesh;
        AssetRef btf;

        bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
        void Serialize(YAML::Emitter &out) const override;
        void Deserialize(const YAML::Node &in) override;

        void CollectAssetRef(std::vector<AssetRef> &list) override;
    };
} // namespace EvoEngine
