#pragma once

#include "Optix7.hpp"
#include "SharedCoordinates.cuh"

namespace evo_engine {
/*! YCbCr to RGB space */
    __device__ inline void YCbCrToRgb(const glm::vec3 &yCbCr, glm::vec3 &rgb) {
        assert(yCbCr[0] >= 0.f);
        assert(yCbCr[1] >= 0.f);
        assert(yCbCr[2] >= 0.f);

        // R
        rgb[0] = yCbCr[0] * 1.1643828f + yCbCr[2] * 1.5960273f - 222.921f;
        // G
        rgb[1] = yCbCr[0] * 1.1643828f - yCbCr[1] * 0.39176172f -
                 yCbCr[2] * 0.81296875f + 135.576f;
        // B
        rgb[2] = yCbCr[0] * 1.1643828f + yCbCr[1] * 2.0172344f - 276.836f;

        if (rgb[0] < 0.f)
            rgb[0] = 0.f;
        if (rgb[1] < 0.f)
            rgb[1] = 0.f;
        if (rgb[2] < 0.f)
            rgb[2] = 0.f;
    }

/*! YCbCr to RGB space */
    __device__ inline void YCbCrToRgbNormalized(const glm::vec3 &yCbCr,
                                                glm::vec3 &rgb) {
        assert(yCbCr[0] >= 0.f);
        assert(yCbCr[1] >= 0.f);
        assert(yCbCr[2] >= 0.f);

        // R
        rgb[0] = yCbCr[0] * 1.1643828f + yCbCr[2] * 1.5960273f - 222.921f / 256.0f;
        // G
        rgb[1] = yCbCr[0] * 1.1643828f - yCbCr[1] * 0.39176172f -
                 yCbCr[2] * 0.81296875f + 135.576f / 256.0f;
        // B
        rgb[2] = yCbCr[0] * 1.1643828f + yCbCr[1] * 2.0172344f - 276.836f / 256.0f;

        if (rgb[0] < 0.f)
            rgb[0] = 0.f;
        if (rgb[1] < 0.f)
            rgb[1] = 0.f;
        if (rgb[2] < 0.f)
            rgb[2] = 0.f;
    }

    __device__ inline int LogLuv2Rgb(const glm::vec3 &luv, glm::vec3 &rgb) {
        /*! \brief Conversion from LogLu'v' (not L*u*v*)  colour space according to
          [Ward98] into RGB colourspace for Observer. = 2degree, Illuminant = D65 */

        // LogLuv -> XYZ

        float Y;

        // HDR
        // This is according to the technical report of Rafal Mantiuk, 2006
        if (luv[0] < 98.381f)
            Y = 0.056968f * luv[0];
        else if (luv[0] < 1204.7f)
            Y = 7.3014e-30f * powf(luv[0] + 884.17f, 9.9872f);
        else
            Y = 32.994f * expf(0.0047811f * luv[0]);

        float X, Z;
        if (luv[2] > 0.f) {
            X = 9.0f / 4.0f * luv[1] / luv[2] * Y;
            Z = Y * (3.0f * 410.f / luv[2] - 5.0f) - X / 3.0f;
        } else {
            assert(luv[1] == 0.f);
            X = Z = 0.f;
        }

        // XYZ -> RGB
        rgb[0] = X * 3.2406f + Y * -1.5372f + Z * -0.4986f;
        rgb[1] = X * -0.9689f + Y * 1.8758f + Z * 0.0415f;
        rgb[2] = X * 0.0557f + Y * -0.2040f + Z * 1.0570f;

        if (rgb[0] < 0.f)
            rgb[0] = 0.f;
        if (rgb[1] < 0.f)
            rgb[1] = 0.f;
        if (rgb[2] < 0.f)
            rgb[2] = 0.f;

        // 11/01/2009 - in fact, this should not be here - but it seems
        // that all UBO HDR images are gamma corrected  - Vlastimil
        if (rgb[0] > 0.0031308f)
            rgb[0] = 1.055f * (float) powf((double) rgb[0], 1.f / 2.4f) - 0.055f;
        else
            rgb[0] = 12.92f * rgb[0];
        if (rgb[1] > 0.0031308f)
            rgb[1] = 1.055f * (float) powf((double) rgb[1], 1.f / 2.4f) - 0.055f;
        else
            rgb[1] = 12.92f * rgb[1];
        if (rgb[2] > 0.0031308f)
            rgb[2] = 1.055f * (float) powf((double) rgb[2], 1.f / 2.4f) - 0.055f;
        else
            rgb[2] = 12.92f * rgb[2];

        return 1;
    } //--- LogLuv2RGB -------------------------------------------

    __device__ inline int LogLuv2RgbNormalized(const glm::vec3 &luv,
                                               glm::vec3 &rgb) {
        /*! \brief Conversion from LogLu'v' (not L*u*v*)  colour space according to
          [Ward98] into RGB colourspace for Observer. = 2degree, Illuminant = D65 */

        // LogLuv -> XYZ

        float Y;

        // HDR
        // This is according to the technical report of Rafal Mantiuk, 2006
        if (luv[0] < 98.381f)
            Y = 0.056968f * luv[0];
        else if (luv[0] < 1204.7f)
            Y = 7.3014e-30f * powf(luv[0] + 884.17f, 9.9872f);
        else
            Y = 32.994f * expf(0.0047811f * luv[0]);

        float X, Z;
        if (luv[2] > 0.f) {
            X = 9.0f / 4.0f * luv[1] / luv[2] * Y;
            Z = Y * (3.0f * 410.f / luv[2] - 5.0f) - X / 3.0f;
        } else {
            assert(luv[1] == 0.f);
            X = Z = 0.f;
        }

        X /= 256.0f;
        Y /= 256.0f;
        Z /= 256.0f;

        // XYZ -> RGB
        rgb[0] = X * 3.2406f + Y * -1.5372f + Z * -0.4986f;
        rgb[1] = X * -0.9689f + Y * 1.8758f + Z * 0.0415f;
        rgb[2] = X * 0.0557f + Y * -0.2040f + Z * 1.0570f;

        if (rgb[0] < 0.f)
            rgb[0] = 0.f;
        if (rgb[1] < 0.f)
            rgb[1] = 0.f;
        if (rgb[2] < 0.f)
            rgb[2] = 0.f;

        // 11/01/2009 - in fact, this should not be here - but it seems
        // that all UBO HDR images are gamma corrected  - Vlastimil
        if (rgb[0] > 0.0031308f)
            rgb[0] = 1.055f * (float) powf((double) rgb[0], 1.f / 2.4f) - 0.055f;
        else
            rgb[0] = 12.92f * rgb[0];
        if (rgb[1] > 0.0031308f)
            rgb[1] = 1.055f * (float) powf((double) rgb[1], 1.f / 2.4f) - 0.055f;
        else
            rgb[1] = 12.92f * rgb[1];
        if (rgb[2] > 0.0031308f)
            rgb[2] = 1.055f * (float) powf((double) rgb[2], 1.f / 2.4f) - 0.055f;
        else
            rgb[2] = 12.92f * rgb[2];

        return 1;
    } //--- LogLuv2RGB -------------------------------------------

// This is user defined model for BTF Compression
    __device__ inline void UserCmToRgb(const glm::vec3 &userColorModelData,
                                       glm::vec3 &rgb,
                                       const SharedCoordinates &tc) {
#ifndef ONLY_ONE_COLOR_SPACE
        if (tc.m_hdrFlag) {
            if (tc.m_codeBtfFlag)
                LogLuv2RgbNormalized(userColorModelData, rgb);
            else
                LogLuv2Rgb(userColorModelData, rgb);
        } else
#endif
        if (tc.m_codeBtfFlag) {
            YCbCrToRgbNormalized(userColorModelData, rgb);
        } else {
            YCbCrToRgb(userColorModelData, rgb);
        }
    }
} // namespace EvoEngine