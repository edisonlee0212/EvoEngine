#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "IAsset.hpp"
#include "filesystem"
#include "BTFBase.cuh"
namespace evo_engine {
    class CompressedBTF : public IAsset {
        std::vector<float> m_sharedCoordinatesBetaAngles;

        std::vector<int> m_pdf6DSlices;
        std::vector<float> m_pdf6DScales;

        std::vector<int> m_pdf4DSlices;
        std::vector<float> m_pdf4DScales;

        std::vector<int> m_pdf3DSlices;
        std::vector<float> m_pdf3DScales;

        std::vector<int> m_indexLuminanceColors;
        std::vector<int> m_pdf2DColors;
        std::vector<float> m_pdf2DScales;
        std::vector<int> m_pdf2DSlices;

        std::vector<int> m_indexAbBasis;

        std::vector<float> m_pdf1DBasis;

        std::vector<float> m_vectorColorBasis;
        void UploadDeviceData();

    public:
        size_t m_version = 0;
        BTFBase m_bTFBase;
        bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
        bool ImportFromFolder(const std::filesystem::path &path);
        void Serialize(YAML::Emitter &out) const override;

        void Deserialize(const YAML::Node &in) override;
    };
}