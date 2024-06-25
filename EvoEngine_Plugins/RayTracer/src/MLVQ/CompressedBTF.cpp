//
// Created by lllll on 10/13/2022.
//

#include "CompressedBTF.hpp"

using namespace evo_engine;

#include "Console.hpp"
#include "Utilities.hpp"

bool ParseFloatData(const std::string &fileName, int &numOfRows, int &numOfCols,
                    float &minValue, float &maxValue,
                    std::vector<float> &data) {
    FILE *fp;
    if ((fp = fopen(fileName.c_str(), "r")) == nullptr) {
        EVOENGINE_ERROR("Error");
        return false;
    }
    int v =
            fscanf(fp, "%d %d %f %f\n", &numOfRows, &numOfCols, &minValue, &maxValue);
    assert(v == 4);
    data.resize(numOfCols * numOfRows);
    for (int row = 0; row < numOfRows; row++) {
        for (int col = 0; col < numOfCols; col++) {
            v = fscanf(fp, "%f ", &data[row * numOfCols + col]);
            assert(v == 1);
        }
        fscanf(fp, "\n");
    }
    fclose(fp);
    return true;
}

bool ParseIntData(const std::string &fileName, int &numOfRows, int &numOfCols,
                  int &minValue, int &maxValue, std::vector<int> &data) {
    FILE *fp;
    if ((fp = fopen(fileName.c_str(), "r")) == nullptr) {
        EVOENGINE_ERROR("Error");
        return false;
    }
    int v =
            fscanf(fp, "%d %d %d %d\n", &numOfRows, &numOfCols, &minValue, &maxValue);
    assert(v == 4);
    data.resize(numOfCols * numOfRows);
    for (int row = 0; row < numOfRows; row++) {
        for (int col = 0; col < numOfCols; col++) {
            v = fscanf(fp, "%d ", &data[row * numOfCols + col]);
            assert(v == 1);
        }
        fscanf(fp, "\n");
    }
    fclose(fp);
    return true;
}

std::string LoadFileAsString(const std::string &path) {
    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        // open files
        file.open(path);
        std::stringstream stream;
        // read file's buffer contents into streams
        stream << file.rdbuf();
        // close file handlers
        file.close();
        // convert stream into string
        return stream.str();
    }
    catch (std::ifstream::failure e) {
        EVOENGINE_ERROR("Load file failed!")
        throw;
    }
}

using namespace evo_engine;

bool CompressedBTF::ImportFromFolder(const std::filesystem::path &path) {
    auto materialDirectoryPath = path.string();
#pragma region Path check
    std::string allMaterialInfo;
    std::string allMaterialInfoPath =
            materialDirectoryPath + "/all_materialInfo.txt";
    bool avoidParFile = false;
    try {
        allMaterialInfo = LoadFileAsString(allMaterialInfoPath);
        avoidParFile = true;
    } catch (std::ifstream::failure e) {
        EVOENGINE_LOG("")
    }
    if (!avoidParFile) {
        EVOENGINE_ERROR("Failed to load BTF material");
        return false;
    }
#pragma endregion
#pragma region Line 82 from ibtfbase.cpp
    m_bTFBase.m_materialOrder = 0;
    m_bTFBase.m_nColor = 0;
    // initial size of arrays
    // How the beta is discretized, either uniformly in degrees
    // or uniformly in cosinus of angle
    m_bTFBase.m_useCosBeta = true;
#pragma endregion
#pragma region Tilemap
    // Since tilemap is not used, the code here is not implemented.
#pragma endregion
#pragma region Scale info
    m_bTFBase.m_mPostScale = 1.0f;
    // Since no material contains the scale.txt is not used, the code here is not
    // implemented.
#pragma endregion
#pragma region material info
    FILE *fp;
    if ((fp = fopen(allMaterialInfoPath.c_str(), "r")) == NULL) {
        EVOENGINE_ERROR("Failed to load BTF material");
        return false;
    }
    // First save the info about BTFbase: name, materials saved, and how saved
    char line[1000];
    int loadMaterials;
    int maxMaterials;
    int flagAllMaterials;
    int flagUse34DviewRep;
    int flagUsePDF2compactRep;

    // First save the info about BTFbase: name, materials saved, and how saved
    if (fscanf(fp, "%s\n%d\n%d\n%d\n%d\n%d\n", &line[0], &loadMaterials,
               &maxMaterials, &flagAllMaterials, &flagUse34DviewRep,
               &flagUsePDF2compactRep) != 6) {
        fclose(fp);
        EVOENGINE_ERROR("File is corrupted for reading basic parameters");
        return false;
    }
    // Here we need to read this information about original data
    int ncolour, nview, nillu, tileSize;
    if (fscanf(fp, "%d\n%d\n%d\n%d\n", &ncolour, &nview, &nillu, &tileSize) !=
        4) {
        fclose(fp);
        EVOENGINE_ERROR(
                "File is corrupted for reading basic parameters about orig database");
        return false;
    }

    // Here we load how parameterization is done
    // It is meant: beta/stepPerBeta, alpha/stepsPerAlpha, theta/stepsPerTheta,
    // phi/stepPerPhi, reserve/reserv, reserve/reserve
    int useCosBetaFlag, stepsPerBeta, tmp3, stepsPerAlpha, tmp5, stepsPerTheta,
            tmp7, stepsPerPhi, tmp9, tmp10, tmp11, tmp12;
    if (fscanf(fp, "%d %d %d %d %d %d %d %d %d %d %d %d\n", &useCosBetaFlag,
               &stepsPerBeta, &tmp3, &stepsPerAlpha, &tmp5, &stepsPerTheta, &tmp7,
               &stepsPerPhi, &tmp9, &tmp10, &tmp11, &tmp12) != 12) {
        fclose(fp);
        EVOENGINE_ERROR("File is corrupted for reading angle parameterization settings\n");
        return false;
    }
    m_bTFBase.m_useCosBeta = useCosBetaFlag ? true : false;
    m_bTFBase.m_numOfBeta = stepsPerBeta;
    assert(m_bTFBase.m_numOfBeta % 2 == 1);
    m_bTFBase.m_numOfAlpha = stepsPerAlpha;
    assert(m_bTFBase.m_numOfAlpha % 2 == 1);
    m_bTFBase.m_numOfTheta = stepsPerTheta;
    assert(m_bTFBase.m_numOfTheta >= 2);
    m_bTFBase.m_numOfPhi = stepsPerPhi;
    assert(m_bTFBase.m_numOfPhi >= 1);
#pragma endregion
#pragma region Create shared variables
    std::vector<float> betaAngles;
    // we always must have odd number of quantization steps per 180 degrees
    if (m_bTFBase.m_useCosBeta) {
        betaAngles.resize(m_bTFBase.m_numOfBeta);
        for (int i = 0; i < m_bTFBase.m_numOfBeta; i++) {
            float sinBeta = -1.0f + 2.0f * i / (m_bTFBase.m_numOfBeta - 1);
            if (sinBeta > 1.0f)
                sinBeta = 1.0f;
            // in degrees
            betaAngles[i] = glm::degrees(glm::asin(sinBeta));
        }
        betaAngles[0] = -90.f;
        betaAngles[(m_bTFBase.m_numOfBeta - 1) / 2] = 0.f;
        betaAngles[m_bTFBase.m_numOfBeta - 1] = 90.f;
    } else {
        float stepBeta = 0.f;
        // uniform quantization in angle
        stepBeta = 180.f / (m_bTFBase.m_numOfBeta - 1);
        betaAngles.resize(m_bTFBase.m_numOfBeta);
        for (int i = 0; i < m_bTFBase.m_numOfBeta; i++) {
            betaAngles[i] = i * stepBeta - 90.f;
        }
        betaAngles[(m_bTFBase.m_numOfBeta - 1) / 2] = 0.f;
        betaAngles[m_bTFBase.m_numOfBeta - 1] = 90.0f;
    }
    // Here we set alpha
    m_bTFBase.m_stepAlpha = 180.f / (m_bTFBase.m_numOfAlpha - 1);
    m_bTFBase.m_stepTheta = 90.0f / (m_bTFBase.m_numOfTheta - 1);
    m_bTFBase.m_stepPhi = 360.0f / m_bTFBase.m_numOfPhi;
    m_bTFBase.m_sharedCoordinates.Set(tmp12, m_bTFBase.m_useCosBeta, m_bTFBase.m_numOfBeta,
                                      m_bTFBase.m_numOfAlpha, m_bTFBase.m_stepAlpha, m_bTFBase.m_numOfTheta,
                                      m_bTFBase.m_stepTheta, m_bTFBase.m_numOfPhi, m_bTFBase.m_stepPhi);
    m_sharedCoordinatesBetaAngles = betaAngles;
#pragma endregion
#pragma region Current settings
    // Here we need to read this information about current material setting
    // where are the starting points for the next search, possibly
    int fPDF1, fAB, fIAB, fPDF2, fPDF2L, fPDF2AB, fPDF3, fPDF34, fPDF4, fRESERVE;
    if (fscanf(fp, "%d %d %d %d %d %d %d %d %d %d\n", &fPDF1, &fAB, &fIAB, &fPDF2,
               &fPDF2L, &fPDF2AB, &fPDF3, &fPDF34, &fPDF4, &fRESERVE) != 10) {
        fclose(fp);
        EVOENGINE_ERROR("File is corrupted for reading starting search settings\n");
        return false;
    }
    // Here we need to save this information about current material setting
    int lsPDF1, lsAB, lsIAB, lsPDF2, lsPDF2L, lsPDF2AB, lsPDF3, lsPDF34, lsPDF4,
            lsRESERVE;
    if (fscanf(fp, "%d %d %d %d %d %d %d %d %d %d\n", &lsPDF1, &lsAB, &lsIAB,
               &lsPDF2, &lsPDF2L, &lsPDF2AB, &lsPDF3, &lsPDF34, &lsPDF4,
               &lsRESERVE) != 10) {
        fclose(fp);
        EVOENGINE_ERROR("File is corrupted for reading starting search points\n");
        return false;
    }

    int metric;
    float baseEps, rPDF1, epsAB, epsIAB, rPDF2, rPDF2L, epsPDF2AB, rPDF3, rPDF34,
            rPDF4, rPDF4b;
    if (fscanf(fp, "%d %f %f %f %f %f %f %f %f %f %f %f\n", &metric, &baseEps,
               &rPDF1, &epsAB, &epsIAB, &rPDF2, &rPDF2L, &epsPDF2AB, &rPDF3,
               &rPDF34, &rPDF4, &rPDF4b) != 12) {
        fclose(fp);
        EVOENGINE_ERROR("File is corrupted for reading epsilon search settings\n");
        return false;
    }
#pragma endregion
#pragma region Load sizes
    // !!!!!! If we have only one database for all materials or
    // we share some databases except PDF6 for all materials
    m_bTFBase.m_use34ViewRepresentation = flagUse34DviewRep;
    m_bTFBase.m_usePdf2CompactRep = flagUsePDF2compactRep;

    if (loadMaterials > maxMaterials)
        loadMaterials = maxMaterials;
    m_bTFBase.m_materialCount = maxMaterials;
    if (flagAllMaterials) {
        m_bTFBase.m_allMaterialsInOneDatabase = true;
    } else {
        m_bTFBase.m_allMaterialsInOneDatabase = false;
    }

#pragma endregion
#pragma region Allocate arrays
    if (!m_bTFBase.m_allMaterialsInOneDatabase && loadMaterials != 1) {
        EVOENGINE_ERROR("Database for multiple materials are not supported!");
        return false;
    }
    // Here we only allow single material, so the array representations in
    // original CompressedBTF lib are not implemented.
#pragma endregion
#pragma region HDR
    std::string materialName;
    std::string inputPath;
    std::string outputPath;
    std::string tempPath;
    float hdrValue = 1.0f;
    int ro, co, pr, pc;
    char l1[1000], l2[1000], l3[1000], l4[1000];
    int hdrFlag = 0;
    if (fscanf(fp, "%s %s %s %s %d %d %d %d %f\n", l1, l2, l3, l4, &ro, &co, &pr,
               &pc, &hdrValue) == 9) {
        // Here we need to allocate the arrays for names
        materialName = std::string(l1);
        inputPath = std::string(l2);
        outputPath = std::string(l3);
        tempPath = std::string(l4);

        if (fabs(hdrValue - 1.0f) < 1e-6 || fabs(hdrValue) < 1e-6) {
            hdrFlag = 0;
            hdrValue = 1.0f;
        } else {
            hdrFlag = 1;
        }
        m_bTFBase.m_sharedCoordinates.m_hdrFlag = hdrFlag;
        m_bTFBase.m_hdr = hdrFlag;
        m_bTFBase.m_hdrValue = hdrValue;
    }
    fclose(fp);
#pragma endregion
#pragma region Load material
    // Note that nrows and ncols are not set during loading !
    std::string fileName =
            materialDirectoryPath + "/" + materialName + "_materialInfo.txt";
    // Now creating PDF6 for each material using common database
    if ((fp = fopen(fileName.c_str(), "r")) == NULL) {
        EVOENGINE_ERROR("Cannot open file" + fileName);
        return false;
    }
    char nameM[200];
    if (fscanf(fp, "%s %s %s %s %d %d %d %d %f\n", &(nameM[0]), l1, l2, l3, &ro,
               &co, &pr, &pc, &hdrValue) != 9) {
        EVOENGINE_ERROR("ERROR:Reading the information about material failed\n");
        fclose(fp);
        return false;
    }
    inputPath = std::string(l1);
    outputPath = std::string(l2);
    tempPath = std::string(l3);
    fclose(fp);
    if (glm::abs(hdrValue - 1.0f) < 1e-6 || glm::abs(hdrValue) < 1e-6) {
        hdrFlag = 0;
        hdrValue = 1.0f;
    } else {
        hdrFlag = 1;
    }

    if (strcmp(nameM, materialName.c_str()) != 0) {
        EVOENGINE_ERROR("Some problem material name in file=" + std::string(nameM) +
                        " other name=" + materialName + "\n");
        return false;
    }
    // Now we can create the database, PDF6 is allocated
    // with right values
    m_bTFBase.m_sharedCoordinates.m_hdrFlag = hdrFlag;
    m_bTFBase.m_hdr = hdrFlag;
    m_bTFBase.m_hdrValue = hdrValue;

    auto &ab = m_bTFBase.m_pdf6.m_pdf4.m_pdf3.m_pdf2.m_iab.m_ab;
    auto &iab = m_bTFBase.m_pdf6.m_pdf4.m_pdf3.m_pdf2.m_iab;
    auto &pdf1 = m_bTFBase.m_pdf6.m_pdf4.m_pdf3.m_pdf2.m_pdf1;
    auto &pdf2 = m_bTFBase.m_pdf6.m_pdf4.m_pdf3.m_pdf2;
    auto &pdf3 = m_bTFBase.m_pdf6.m_pdf4.m_pdf3;
    auto &pdf4 = m_bTFBase.m_pdf6.m_pdf4;
    pdf1.Init(m_bTFBase.m_numOfBeta);
    ab.Init();
    iab.Init(m_bTFBase.m_numOfBeta);
    pdf2.Init(m_bTFBase.m_numOfAlpha);
    pdf3.Init(m_bTFBase.m_numOfTheta);
    pdf4.Init(m_bTFBase.m_numOfPhi);
    m_bTFBase.m_pdf6.Init(pr, pc, ro, co, m_bTFBase.m_nColor);

#pragma region Load Data
    std::string prefix = materialDirectoryPath + "/" + materialName;
    int minIntVal, maxIntVal;
    float minFloatVal, maxFloatVal;

    ParseIntData(prefix + "_PDF6Dslices.txt", m_bTFBase.m_pdf6.m_numOfRows,
                 m_bTFBase.m_pdf6.m_numOfCols, minIntVal, maxIntVal, m_pdf6DSlices);
    ParseFloatData(prefix + "_PDF6Dscale.txt", m_bTFBase.m_pdf6.m_numOfRows,
                   m_bTFBase.m_pdf6.m_numOfCols, minFloatVal, maxFloatVal, m_pdf6DScales);

    prefix = materialDirectoryPath + "/" + "all";

    ParseFloatData(prefix + "_PDF1Dslice.txt", pdf1.m_numOfPdf1D,
                   pdf1.m_numOfBeta, minFloatVal, maxFloatVal, m_pdf1DBasis);

    ParseFloatData(prefix + "_colors.txt", ab.m_numOfColors, ab.m_numOfChannels,
                   minFloatVal, maxFloatVal, m_vectorColorBasis);

    ParseIntData(prefix + "_indexAB.txt", iab.m_numOfIndexSlices, iab.m_numOfBeta,
                 minIntVal, maxIntVal, m_indexAbBasis);

    ParseIntData(prefix + "_PDF2Dcolours.txt", pdf2.m_color.m_numOfPdf2D,
                 pdf2.m_color.m_numOfAlpha, minIntVal, maxIntVal, m_pdf2DColors);

    ParseIntData(prefix + "_PDF2Dslices.txt", pdf2.m_luminance.m_numOfPdf2D,
                 pdf2.m_luminance.m_numOfAlpha, minIntVal, maxIntVal, m_pdf2DSlices);
    ParseFloatData(prefix + "_PDF2Dscale.txt", pdf2.m_luminance.m_numOfPdf2D,
                   pdf2.m_luminance.m_numOfAlpha, minFloatVal, maxFloatVal,
                   m_pdf2DScales);

    ParseIntData(prefix + "_PDF2Dindices.txt", pdf2.m_numOfPdf2D,
                 pdf2.m_lengthOfSlice, minIntVal, maxIntVal, m_indexLuminanceColors);

    ParseFloatData(prefix + "_PDF3Dscale.txt", pdf3.m_numOfPdf3D,
                   pdf3.m_numOfTheta, minFloatVal, maxFloatVal, m_pdf3DScales);

    ParseIntData(prefix + "_PDF3Dslices.txt", pdf3.m_numOfPdf3D,
                 pdf3.m_numOfTheta, minIntVal, maxIntVal, m_pdf3DSlices);

    ParseFloatData(prefix + "_PDF4Dscale.txt", pdf4.m_numOfPdf4D, pdf4.m_numOfPhi,
                   minFloatVal, maxFloatVal, m_pdf4DScales);

    ParseIntData(prefix + "_PDF4Dslices.txt", pdf4.m_numOfPdf4D, pdf4.m_numOfPhi,
                 minIntVal, maxIntVal, m_pdf4DSlices);



#pragma endregion
    if (m_bTFBase.m_hdr) {
        m_bTFBase.m_multiplier = m_bTFBase.m_hdrValue;
    } else {
        m_bTFBase.m_multiplier = 1.0f;
    }
    UploadDeviceData();
    return true; // OK - database loaded, or at least partially
#pragma endregion
}

bool CompressedBTF::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
    bool changed = false;
    FileUtils::OpenFolder("Import Database", [&](const std::filesystem::path &path) {
        try {
            bool succeed = ImportFromFolder(path);
            if (succeed) changed = true;
            EVOENGINE_LOG((std::string("BTF Material import ") + (succeed ? "succeed" : "failed")))
        } catch (const std::exception &e) {
            EVOENGINE_ERROR(std::string(e.what()))
        }
    }, false);

    if (m_bTFBase.m_hasData) {
        if (ImGui::DragFloat("Multiplier", &m_bTFBase.m_multiplier, 1.0f)) {
            changed = true;
        }

        if (ImGui::DragFloat("TexCoord Multiplier", &m_bTFBase.m_texCoordMultiplier, 0.1f)) {
            changed = true;
        }

        if (ImGui::Checkbox("HDR", &m_bTFBase.m_hdr)) {
            changed = true;
        }

        if (ImGui::DragFloat("HDR Value", &m_bTFBase.m_hdrValue, 0.01f)) {
            changed = true;
        }
        if (ImGui::DragFloat("Gamma Value", &m_bTFBase.m_gamma, 0.01f)) {
            changed = true;
        }
    }
    return changed;
}


void SerializeSharedCoordinates(const SharedCoordinates &sharedCoordinates, YAML::Emitter &out) {
    out << YAML::Key << "m_useCosBeta" << YAML::Value << sharedCoordinates.m_useCosBeta;

    out << YAML::Key << "m_numOfBeta" << YAML::Value << sharedCoordinates.m_numOfBeta;
    out << YAML::Key << "m_stepAlpha" << YAML::Value << sharedCoordinates.m_stepAlpha;
    out << YAML::Key << "m_numOfAlpha" << YAML::Value << sharedCoordinates.m_numOfAlpha;
    out << YAML::Key << "m_stepTheta" << YAML::Value << sharedCoordinates.m_stepTheta;
    out << YAML::Key << "m_numOfTheta" << YAML::Value << sharedCoordinates.m_numOfTheta;
    out << YAML::Key << "m_stepPhi" << YAML::Value << sharedCoordinates.m_stepPhi;
    out << YAML::Key << "m_numOfPhi" << YAML::Value << sharedCoordinates.m_numOfPhi;

    out << YAML::Key << "m_beta" << YAML::Value << sharedCoordinates.m_beta;
    out << YAML::Key << "m_alpha" << YAML::Value << sharedCoordinates.m_alpha;
    out << YAML::Key << "m_theta" << YAML::Value << sharedCoordinates.m_theta;
    out << YAML::Key << "m_phi" << YAML::Value << sharedCoordinates.m_phi;

    out << YAML::Key << "m_currentBetaLowBound" << YAML::Value << sharedCoordinates.m_currentBetaLowBound;
    out << YAML::Key << "m_weightBeta" << YAML::Value << sharedCoordinates.m_weightBeta;
    out << YAML::Key << "m_wMinBeta2" << YAML::Value << sharedCoordinates.m_wMinBeta2;

    out << YAML::Key << "m_currentAlphaLowBound" << YAML::Value << sharedCoordinates.m_currentAlphaLowBound;
    out << YAML::Key << "m_weightAlpha" << YAML::Value << sharedCoordinates.m_weightAlpha;
    out << YAML::Key << "m_wMinAlpha2" << YAML::Value << sharedCoordinates.m_wMinAlpha2;

    out << YAML::Key << "m_currentThetaLowBound" << YAML::Value << sharedCoordinates.m_currentThetaLowBound;
    out << YAML::Key << "m_weightTheta" << YAML::Value << sharedCoordinates.m_weightTheta;
    out << YAML::Key << "m_wMinTheta2" << YAML::Value << sharedCoordinates.m_wMinTheta2;

    out << YAML::Key << "m_currentPhiLowBound" << YAML::Value << sharedCoordinates.m_currentPhiLowBound;
    out << YAML::Key << "m_weightPhi" << YAML::Value << sharedCoordinates.m_weightPhi;

    out << YAML::Key << "m_scale" << YAML::Value << sharedCoordinates.m_scale;

    out << YAML::Key << "m_hdrFlag" << YAML::Value << sharedCoordinates.m_hdrFlag;
    out << YAML::Key << "m_codeBtfFlag" << YAML::Value << sharedCoordinates.m_codeBtfFlag;
}

void DeserializeSharedCoordinates(SharedCoordinates &target, const YAML::Node &in) {
    if (in["m_useCosBeta"]) target.m_useCosBeta = in["m_useCosBeta"].as<bool>();

    if (in["m_numOfBeta"]) target.m_numOfBeta = in["m_numOfBeta"].as<int>();
    if (in["m_stepAlpha"]) target.m_stepAlpha = in["m_stepAlpha"].as<float>();
    if (in["m_numOfAlpha"]) target.m_numOfAlpha = in["m_numOfAlpha"].as<int>();
    if (in["m_stepTheta"]) target.m_stepTheta = in["m_stepTheta"].as<float>();
    if (in["m_numOfTheta"]) target.m_numOfTheta = in["m_numOfTheta"].as<int>();
    if (in["m_stepPhi"]) target.m_stepPhi = in["m_stepPhi"].as<float>();
    if (in["m_numOfPhi"]) target.m_numOfPhi = in["m_numOfPhi"].as<int>();

    if (in["m_beta"]) target.m_beta = in["m_beta"].as<float>();
    if (in["m_alpha"]) target.m_alpha = in["m_alpha"].as<float>();
    if (in["m_theta"]) target.m_theta = in["m_theta"].as<float>();
    if (in["m_phi"]) target.m_phi = in["m_phi"].as<float>();

    if (in["m_currentBetaLowBound"]) target.m_currentBetaLowBound = in["m_currentBetaLowBound"].as<int>();
    if (in["m_weightBeta"]) target.m_weightBeta = in["m_weightBeta"].as<float>();
    if (in["m_wMinBeta2"]) target.m_wMinBeta2 = in["m_wMinBeta2"].as<float>();

    if (in["m_currentAlphaLowBound"]) target.m_currentAlphaLowBound = in["m_currentAlphaLowBound"].as<int>();
    if (in["m_weightAlpha"]) target.m_weightAlpha = in["m_weightAlpha"].as<float>();
    if (in["m_wMinAlpha2"]) target.m_wMinAlpha2 = in["m_wMinAlpha2"].as<float>();

    if (in["m_currentThetaLowBound"]) target.m_currentThetaLowBound = in["m_currentThetaLowBound"].as<int>();
    if (in["m_weightTheta"]) target.m_weightTheta = in["m_weightTheta"].as<float>();
    if (in["m_wMinTheta2"]) target.m_wMinTheta2 = in["m_wMinTheta2"].as<float>();

    if (in["m_currentPhiLowBound"]) target.m_currentPhiLowBound = in["m_currentPhiLowBound"].as<int>();
    if (in["m_weightPhi"]) target.m_weightPhi = in["m_weightPhi"].as<float>();

    if (in["m_scale"]) target.m_scale = in["m_scale"].as<float>();

    if (in["m_hdrFlag"]) target.m_hdrFlag = in["m_hdrFlag"].as<bool>();
    if (in["m_codeBtfFlag"]) target.m_codeBtfFlag = in["m_codeBtfFlag"].as<bool>();
}

void SerializeVectorColor(const VectorColor &target, YAML::Emitter &out) {
    out << YAML::Key << "m_startIndex" << YAML::Value << target.m_startIndex;
    out << YAML::Key << "m_numOfChannels" << YAML::Value << target.m_numOfChannels;
    out << YAML::Key << "m_numOfColors" << YAML::Value << target.m_numOfColors;

}

void DeserializeVectorColor(VectorColor &target, const YAML::Node &in) {
    if (in["m_startIndex"]) target.m_startIndex = in["m_startIndex"].as<int>();
    if (in["m_numOfChannels"]) target.m_numOfChannels = in["m_numOfChannels"].as<int>();
    if (in["m_numOfColors"]) target.m_numOfColors = in["m_numOfColors"].as<int>();
}

void SerializeIndexAB(const IndexAB &target, YAML::Emitter &out) {
    out << YAML::Key << "m_numOfIndexSlices" << YAML::Value << target.m_numOfIndexSlices;
    out << YAML::Key << "m_numOfBeta" << YAML::Value << target.m_numOfBeta;
    //m_ab
    out << YAML::Key << "m_ab" << YAML::Value << YAML::BeginMap;
    SerializeVectorColor(target.m_ab, out);
    out << YAML::EndMap;
}

void DeserializeIndexAB(IndexAB &target, const YAML::Node &in) {
    if (in["m_numOfIndexSlices"]) target.m_numOfIndexSlices = in["m_numOfIndexSlices"].as<int>();
    if (in["m_numOfBeta"]) target.m_numOfBeta = in["m_numOfBeta"].as<int>();
    if (in["m_ab"]) DeserializeVectorColor(target.m_ab, in["m_ab"]);
}

void SerializePDF1D(const PDF1D &target, YAML::Emitter &out) {
    out << YAML::Key << "m_numOfBeta" << YAML::Value << target.m_numOfBeta;
    out << YAML::Key << "m_numOfPdf1D" << YAML::Value << target.m_numOfPdf1D;
}

void DeserializePDF1D(PDF1D &target, const YAML::Node &in) {
    if (in["m_numOfBeta"]) target.m_numOfBeta = in["m_numOfBeta"].as<int>();
    if (in["m_numOfPdf1D"]) target.m_numOfPdf1D = in["m_numOfPdf1D"].as<int>();
}

void SerializePDF2D(const PDF2D &target, YAML::Emitter &out) {
    out << YAML::Key << "m_numOfPdf2D" << YAML::Value << target.m_numOfPdf2D;
    out << YAML::Key << "m_size2D" << YAML::Value << target.m_size2D;
    out << YAML::Key << "m_lengthOfSlice" << YAML::Value << target.m_lengthOfSlice;

    out << YAML::Key << "m_color.m_numOfPdf2D" << YAML::Value << target.m_color.m_numOfPdf2D;
    out << YAML::Key << "m_color.m_numOfAlpha" << YAML::Value << target.m_color.m_numOfAlpha;
    out << YAML::Key << "m_color.m_size2D" << YAML::Value << target.m_color.m_size2D;

    out << YAML::Key << "m_luminance.m_numOfPdf2D" << YAML::Value << target.m_luminance.m_numOfPdf2D;
    out << YAML::Key << "m_luminance.m_numOfAlpha" << YAML::Value << target.m_luminance.m_numOfAlpha;

    //m_iab
    out << YAML::Key << "m_iab" << YAML::Value << YAML::BeginMap;
    SerializeIndexAB(target.m_iab, out);
    out << YAML::EndMap;
    //m_pdf1
    out << YAML::Key << "m_pdf1" << YAML::Value << YAML::BeginMap;
    SerializePDF1D(target.m_pdf1, out);
    out << YAML::EndMap;
}

void DeserializePDF2D(PDF2D &target, const YAML::Node &in) {
    if (in["m_numOfPdf2D"]) target.m_numOfPdf2D = in["m_numOfPdf2D"].as<int>();
    if (in["m_size2D"]) target.m_size2D = in["m_size2D"].as<int>();
    if (in["m_lengthOfSlice"]) target.m_lengthOfSlice = in["m_lengthOfSlice"].as<int>();

    if (in["m_color.m_numOfPdf2D"]) target.m_color.m_numOfPdf2D = in["m_color.m_numOfPdf2D"].as<int>();
    if (in["m_color.m_numOfAlpha"]) target.m_color.m_numOfAlpha = in["m_color.m_numOfAlpha"].as<int>();
    if (in["m_color.m_size2D"]) target.m_color.m_size2D = in["m_color.m_size2D"].as<int>();

    if (in["m_luminance.m_numOfPdf2D"]) target.m_luminance.m_numOfPdf2D = in["m_luminance.m_numOfPdf2D"].as<int>();
    if (in["m_luminance.m_numOfAlpha"]) target.m_luminance.m_numOfAlpha = in["m_luminance.m_numOfAlpha"].as<int>();


    if (in["m_iab"]) DeserializeIndexAB(target.m_iab, in["m_iab"]);
    if (in["m_pdf1"]) DeserializePDF1D(target.m_pdf1, in["m_pdf1"]);

}

template<typename T>
void SerializePDF3D(const PDF3D<T> &target, YAML::Emitter &out) {
    out << YAML::Key << "m_numOfPdf3D" << YAML::Value << target.m_numOfPdf3D;
    out << YAML::Key << "m_numOfTheta" << YAML::Value << target.m_numOfTheta;

    //m_pdf2
    out << YAML::Key << "m_pdf2" << YAML::Value << YAML::BeginMap;
    SerializePDF2D(target.m_pdf2, out);
    out << YAML::EndMap;
}

template<typename T>
void DeserializePDF3D(PDF3D<T> &target, const YAML::Node &in) {
    if (in["m_numOfPdf3D"]) target.m_numOfPdf3D = in["m_numOfPdf3D"].as<int>();
    if (in["m_numOfTheta"]) target.m_numOfTheta = in["m_numOfTheta"].as<int>();

    if (in["m_pdf2"]) DeserializePDF2D(target.m_pdf2, in["m_pdf2"]);
}

template<typename T>
void SerializePDF4D(const PDF4D<T> &target, YAML::Emitter &out) {
    out << YAML::Key << "m_numOfPdf4D" << YAML::Value << target.m_numOfPdf4D;
    out << YAML::Key << "m_numOfPhi" << YAML::Value << target.m_numOfPhi;
    out << YAML::Key << "m_stepPhi" << YAML::Value << target.m_stepPhi;

    //m_pdf3
    out << YAML::Key << "m_pdf3" << YAML::Value << YAML::BeginMap;
    SerializePDF3D(target.m_pdf3, out);
    out << YAML::EndMap;
}

template<typename T>
void DeserializePDF4D(PDF4D<T> &target, const YAML::Node &in) {
    if (in["m_numOfPdf4D"]) target.m_numOfPdf4D = in["m_numOfPdf4D"].as<int>();
    if (in["m_numOfPhi"]) target.m_numOfPhi = in["m_numOfPhi"].as<int>();
    if (in["m_stepPhi"]) target.m_stepPhi = in["m_stepPhi"].as<float>();

    if (in["m_pdf3"]) DeserializePDF3D(target.m_pdf3, in["m_pdf3"]);
}

template<typename T>
void SerializePDF6D(const PDF6D<T> &target, YAML::Emitter &out) {
    out << YAML::Key << "m_numOfRows" << YAML::Value << target.m_numOfRows;
    out << YAML::Key << "m_numOfCols" << YAML::Value << target.m_numOfCols;
    out << YAML::Key << "m_rowsOffset" << YAML::Value << target.m_rowsOffset;
    out << YAML::Key << "m_colsOffset" << YAML::Value << target.m_colsOffset;
    out << YAML::Key << "m_colorAmount" << YAML::Value << target.m_colorAmount;

    //m_pdf4
    out << YAML::Key << "m_pdf4" << YAML::Value << YAML::BeginMap;
    SerializePDF4D(target.m_pdf4, out);
    out << YAML::EndMap;
}

template<typename T>
void DeserializePDF6D(PDF6D<T> &target, const YAML::Node &in) {
    if (in["m_numOfRows"]) target.m_numOfRows = in["m_numOfRows"].as<int>();
    if (in["m_numOfCols"]) target.m_numOfCols = in["m_numOfCols"].as<int>();
    if (in["m_rowsOffset"]) target.m_rowsOffset = in["m_rowsOffset"].as<int>();

    if (in["m_colsOffset"]) target.m_colsOffset = in["m_colsOffset"].as<int>();

    if (in["m_colorAmount"]) target.m_colorAmount = in["m_colorAmount"].as<int>();

    if (in["m_pdf4"]) DeserializePDF4D(target.m_pdf4, in["m_pdf4"]);
}

void SerializeBTFBase(const BTFBase &target, YAML::Emitter &out) {
    out << YAML::Key << "m_sharedCoordinates" << YAML::Value << YAML::BeginMap;
    SerializeSharedCoordinates(target.m_sharedCoordinates, out);
    out << YAML::EndMap;

    out << YAML::Key << "m_pdf6" << YAML::Value << YAML::BeginMap;
    SerializePDF6D(target.m_pdf6, out);
    out << YAML::EndMap;


    out << YAML::Key << "m_hdr" << YAML::Value << target.m_hdr;
    out << YAML::Key << "m_hdrValue" << YAML::Value << target.m_hdrValue;

    out << YAML::Key << "m_multiplier" << YAML::Value << target.m_multiplier;
    out << YAML::Key << "m_texCoordMultiplier" << YAML::Value << target.m_texCoordMultiplier;
    out << YAML::Key << "m_gamma" << YAML::Value << target.m_gamma;

    out << YAML::Key << "m_materialOrder" << YAML::Value << target.m_materialOrder;
    out << YAML::Key << "m_nColor" << YAML::Value << target.m_nColor;

    out << YAML::Key << "m_useCosBeta" << YAML::Value << target.m_useCosBeta;
    out << YAML::Key << "m_mPostScale" << YAML::Value << target.m_mPostScale;

    out << YAML::Key << "m_numOfBeta" << YAML::Value << target.m_numOfBeta;
    out << YAML::Key << "m_numOfAlpha" << YAML::Value << target.m_numOfAlpha;
    out << YAML::Key << "m_numOfTheta" << YAML::Value << target.m_numOfTheta;
    out << YAML::Key << "m_numOfPhi" << YAML::Value << target.m_numOfPhi;

    out << YAML::Key << "m_stepAlpha" << YAML::Value << target.m_stepAlpha;
    out << YAML::Key << "m_stepTheta" << YAML::Value << target.m_stepTheta;
    out << YAML::Key << "m_stepPhi" << YAML::Value << target.m_stepPhi;

    out << YAML::Key << "m_allMaterialsInOneDatabase" << YAML::Value << target.m_allMaterialsInOneDatabase;
    out << YAML::Key << "m_use34ViewRepresentation" << YAML::Value << target.m_use34ViewRepresentation;
    out << YAML::Key << "m_usePdf2CompactRep" << YAML::Value << target.m_usePdf2CompactRep;

    out << YAML::Key << "m_materialCount" << YAML::Value << target.m_materialCount;
}

void DeserializeBTFBase(BTFBase &target, const YAML::Node &in) {
    if (in["m_sharedCoordinates"]) DeserializeSharedCoordinates(target.m_sharedCoordinates, in["m_sharedCoordinates"]);
    if (in["m_pdf6"]) DeserializePDF6D(target.m_pdf6, in["m_pdf6"]);

    if (in["m_hdr"]) target.m_hdr = in["m_hdr"].as<bool>();
    if (in["m_hdrValue"]) target.m_hdrValue = in["m_hdrValue"].as<float>();

    if (in["m_multiplier"]) target.m_multiplier = in["m_multiplier"].as<float>();
    if (in["m_texCoordMultiplier"]) target.m_texCoordMultiplier = in["m_texCoordMultiplier"].as<float>();
    if (in["m_gamma"]) target.m_gamma = in["m_gamma"].as<float>();

    if (in["m_materialOrder"]) target.m_materialOrder = in["m_materialOrder"].as<int>();
    if (in["m_nColor"]) target.m_nColor = in["m_nColor"].as<int>();

    if (in["m_useCosBeta"]) target.m_useCosBeta = in["m_useCosBeta"].as<bool>();
    if (in["m_mPostScale"]) target.m_mPostScale = in["m_mPostScale"].as<float>();

    if (in["m_numOfBeta"]) target.m_numOfBeta = in["m_numOfBeta"].as<int>();
    if (in["m_numOfAlpha"]) target.m_numOfAlpha = in["m_numOfAlpha"].as<int>();
    if (in["m_numOfTheta"]) target.m_numOfTheta = in["m_numOfTheta"].as<int>();
    if (in["m_numOfPhi"]) target.m_numOfPhi = in["m_numOfPhi"].as<int>();

    if (in["m_stepAlpha"]) target.m_stepAlpha = in["m_stepAlpha"].as<float>();
    if (in["m_stepTheta"]) target.m_stepTheta = in["m_stepTheta"].as<float>();
    if (in["m_stepPhi"]) target.m_stepPhi = in["m_stepPhi"].as<float>();

    if (in["m_allMaterialsInOneDatabase"]) target.m_allMaterialsInOneDatabase = in["m_allMaterialsInOneDatabase"].as<bool>();
    if (in["m_use34ViewRepresentation"]) target.m_use34ViewRepresentation = in["m_use34ViewRepresentation"].as<bool>();
    if (in["m_usePdf2CompactRep"]) target.m_usePdf2CompactRep = in["m_usePdf2CompactRep"].as<bool>();

    if (in["m_materialCount"]) target.m_materialCount = in["m_materialCount"].as<int>();
}

template<typename T>
void LoadList(const std::string &name, const YAML::Node &in, std::vector<T> &target) {
    if (in[name]) {
        const auto& data = in[name].as<YAML::Binary>();
        target.resize(data.size() / sizeof(T));
        std::memcpy(target.data(), data.data(), data.size());
    }
}

template<typename T>
void SaveList(const std::string &name, YAML::Emitter &out, const std::vector<T> &target) {
    if (!target.empty()) {
        out << YAML::Key << name << YAML::Value
            << YAML::Binary((const unsigned char *) target.data(), target.size() * sizeof(T));
    }
}

void CompressedBTF::Serialize(YAML::Emitter &out) const {
    if (m_bTFBase.m_hasData) {
        out << YAML::Key << "m_bTFBase" << YAML::Value << YAML::BeginMap;
        SerializeBTFBase(m_bTFBase, out);
        out << YAML::EndMap;

        SaveList("m_sharedCoordinatesBetaAngles", out, m_sharedCoordinatesBetaAngles);

        SaveList("m_pdf6DSlices", out, m_pdf6DSlices);
        SaveList("m_pdf6DScales", out, m_pdf6DScales);

        SaveList("m_pdf4DSlices", out, m_pdf4DSlices);
        SaveList("m_pdf4DScales", out, m_pdf4DScales);

        SaveList("m_pdf3DSlices", out, m_pdf3DSlices);
        SaveList("m_pdf3DScales", out, m_pdf3DScales);

        SaveList("m_indexLuminanceColors", out, m_indexLuminanceColors);
        SaveList("m_pdf2DColors", out, m_pdf2DColors);
        SaveList("m_pdf2DScales", out, m_pdf2DScales);
        SaveList("m_pdf2DSlices", out, m_pdf2DSlices);

        SaveList("m_indexAbBasis", out, m_indexAbBasis);

        SaveList("m_pdf1DBasis", out, m_pdf1DBasis);
        SaveList("m_vectorColorBasis", out, m_vectorColorBasis);

    }
}

void CompressedBTF::Deserialize(const YAML::Node &in) {
    m_bTFBase.m_hasData = false;
    if (in["m_bTFBase"]) {
        DeserializeBTFBase(m_bTFBase, in["m_bTFBase"]);

        LoadList("m_sharedCoordinatesBetaAngles", in, m_sharedCoordinatesBetaAngles);

        LoadList("m_pdf6DSlices", in, m_pdf6DSlices);
        LoadList("m_pdf6DScales", in, m_pdf6DScales);

        LoadList("m_pdf4DSlices", in, m_pdf4DSlices);
        LoadList("m_pdf4DScales", in, m_pdf4DScales);

        LoadList("m_pdf3DSlices", in, m_pdf3DSlices);
        LoadList("m_pdf3DScales", in, m_pdf3DScales);

        LoadList("m_indexLuminanceColors", in, m_indexLuminanceColors);
        LoadList("m_pdf2DColors", in, m_pdf2DColors);
        LoadList("m_pdf2DScales", in, m_pdf2DScales);
        LoadList("m_pdf2DSlices", in, m_pdf2DSlices);

        LoadList("m_indexAbBasis", in, m_indexAbBasis);

        LoadList("m_pdf1DBasis", in, m_pdf1DBasis);
        LoadList("m_vectorColorBasis", in, m_vectorColorBasis);


        UploadDeviceData();

    }
    saved_ = true;
}

void CompressedBTF::UploadDeviceData() {
    m_bTFBase.m_sharedCoordinates.m_betaAnglesBuffer.Upload(m_sharedCoordinatesBetaAngles);
    m_bTFBase.m_sharedCoordinates.m_deviceBetaAngles =
            reinterpret_cast<float *>(m_bTFBase.m_sharedCoordinates.m_betaAnglesBuffer.DevicePointer());

    auto &pdf6 = m_bTFBase.m_pdf6;
    pdf6.m_pdf6DSlicesBuffer.Upload(m_pdf6DSlices);
    pdf6.m_pdf6DScalesBuffer.Upload(m_pdf6DScales);
    pdf6.m_devicePdf6DSlices =
            reinterpret_cast<int *>(pdf6.m_pdf6DSlicesBuffer.DevicePointer());
    pdf6.m_devicePdf6DScales =
            reinterpret_cast<float *>(pdf6.m_pdf6DScalesBuffer.DevicePointer());

    auto &pdf4 = pdf6.m_pdf4;
    pdf4.m_pdf4DScalesBuffer.Upload(m_pdf4DScales);
    pdf4.m_pdf4DSlicesBuffer.Upload(m_pdf4DSlices);
    pdf4.m_devicePdf4DScales =
            reinterpret_cast<float *>(pdf4.m_pdf4DScalesBuffer.DevicePointer());
    pdf4.m_devicePdf4DSlices =
            reinterpret_cast<int *>(pdf4.m_pdf4DSlicesBuffer.DevicePointer());

    auto &pdf3 = pdf4.m_pdf3;
    pdf3.m_pdf3DScalesBuffer.Upload(m_pdf3DScales);
    pdf3.m_pdf3DSlicesBuffer.Upload(m_pdf3DSlices);
    pdf3.m_devicePdf3DScales =
            reinterpret_cast<float *>(pdf3.m_pdf3DScalesBuffer.DevicePointer());
    pdf3.m_devicePdf3DSlices =
            reinterpret_cast<int *>(pdf3.m_pdf3DSlicesBuffer.DevicePointer());

    auto &pdf2 = pdf3.m_pdf2;
    pdf2.m_indexLuminanceColorBuffer.Upload(m_indexLuminanceColors);
    pdf2.m_deviceIndexLuminanceColor =
            reinterpret_cast<int *>(pdf2.m_indexLuminanceColorBuffer.DevicePointer());

    auto &color = pdf2.m_color;
    color.m_pdf2DColorsBuffer.Upload(m_pdf2DColors);
    color.m_devicePdf2DColors =
            reinterpret_cast<int *>(color.m_pdf2DColorsBuffer.DevicePointer());

    auto &luminance = pdf2.m_luminance;
    luminance.m_pdf2DSlicesBuffer.Upload(m_pdf2DSlices);
    luminance.m_pdf2DScalesBuffer.Upload(m_pdf2DScales);
    luminance.m_devicePdf2DSlices = reinterpret_cast<int *>(
            luminance.m_pdf2DSlicesBuffer.DevicePointer());
    luminance.m_devicePdf2DScales = reinterpret_cast<float *>(
            luminance.m_pdf2DScalesBuffer.DevicePointer());

    auto &iab = pdf2.m_iab;
    iab.m_indexAbBasisBuffer.Upload(m_indexAbBasis);
    iab.m_deviceIndexAbBasis =
            reinterpret_cast<int *>(iab.m_indexAbBasisBuffer.DevicePointer());

    auto &pdf1 = pdf2.m_pdf1;
    pdf1.m_pdf1DBasisBuffer.Upload(m_pdf1DBasis);
    pdf1.m_devicePdf1DBasis =
            reinterpret_cast<float *>(pdf1.m_pdf1DBasisBuffer.DevicePointer());

    auto &ab = iab.m_ab;
    ab.m_vectorColorBasisBuffer.Upload(m_vectorColorBasis);
    ab.m_deviceVectorColorBasis =
            reinterpret_cast<float *>(ab.m_vectorColorBasisBuffer.DevicePointer());

    m_bTFBase.m_hasData = true;
}


