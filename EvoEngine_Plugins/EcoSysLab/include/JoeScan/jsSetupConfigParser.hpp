/**
 * Copyright (c) JoeScan Inc. All Rights Reserved.
 *
 * Licensed under the BSD 3 Clause License. See LICENSE.txt in the project
 * root for license information.
 */

#ifndef JOESCAN_JSSETUP_CONFIG_PARSER_H
#define JOESCAN_JSSETUP_CONFIG_PARSER_H

#include <fstream>
#include <string>
#include "joescan_pinchot.h"
#include "json.h"
using namespace treeio;
namespace joescan {

/// Internal function
inline jsCamera _Str2Cam(std::string str) {
  if (0 == str.compare("CameraA")) {
    return JS_CAMERA_A;
  } else if (0 == str.compare("CameraB")) {
    return JS_CAMERA_B;
  }

  return JS_CAMERA_INVALID;
}

/// Internal function
inline jsLaser _Str2Las(std::string str) {
  if (0 == str.compare("Laser1")) {
    return JS_LASER_1;
  } else if (0 == str.compare("Laser2")) {
    return JS_LASER_2;
  } else if (0 == str.compare("Laser3")) {
    return JS_LASER_3;
  } else if (0 == str.compare("Laser4")) {
    return JS_LASER_4;
  } else if (0 == str.compare("Laser5")) {
    return JS_LASER_5;
  } else if (0 == str.compare("Laser6")) {
    return JS_LASER_6;
  } else if (0 == str.compare("Laser7")) {
    return JS_LASER_7;
  } else if (0 == str.compare("Laser8")) {
    return JS_LASER_8;
  }

  return JS_LASER_INVALID;
}

/// Internal function
inline jsScanHeadType _Str2Type(std::string str) {
  if (0 == str.compare("JS50WX")) {
    return JS_SCAN_HEAD_JS50WX;
  } else if (0 == str.compare("JS50WSC")) {
    return JS_SCAN_HEAD_JS50WSC;
  } else if (0 == str.compare("JS50MX")) {
    return JS_SCAN_HEAD_JS50MX;
  } else if (0 == str.compare("JS50X6B20")) {
    return JS_SCAN_HEAD_JS50X6B20;
  } else if (0 == str.compare("JS50X6B30")) {
    return JS_SCAN_HEAD_JS50X6B30;
  } else if (0 == str.compare("JS50Z820")) {
    return JS_SCAN_HEAD_JS50Z820;
  } else if (0 == str.compare("JS50Z830")) {
    return JS_SCAN_HEAD_JS50Z830;
  }

  return JS_SCAN_HEAD_INVALID_TYPE;
}

/// Internal function
inline jsCableOrientation _Str2Cable(std::string str) {
  if (0 == str.compare("CableIsUpstream")) {
    return JS_CABLE_ORIENTATION_UPSTREAM;
  } else if (0 == str.compare("CableIsDownstream")) {
    return JS_CABLE_ORIENTATION_DOWNSTREAM;
  }

  return JS_CABLE_ORIENTATION_INVALID;
}

/// Internal function
inline std::string _Cam2Str(jsCamera camera) {
  if (JS_CAMERA_A == camera) {
    return "CameraA";
  } else if (JS_CAMERA_B == camera) {
    return "CameraB";
  }

  return "CameraInvalid";
}

/// Internal function
inline std::string _Las2Str(jsLaser laser) {
  if (JS_LASER_1 == laser) {
    return "Laser1";
  } else if (JS_LASER_2 == laser) {
    return "Laser2";
  } else if (JS_LASER_3 == laser) {
    return "Laser3";
  } else if (JS_LASER_4 == laser) {
    return "Laser4";
  } else if (JS_LASER_5 == laser) {
    return "Laser5";
  } else if (JS_LASER_6 == laser) {
    return "Laser6";
  } else if (JS_LASER_7 == laser) {
    return "Laser7";
  } else if (JS_LASER_8 == laser) {
    return "Laser8";
  }

  return "LaserInvalid";
}

/// Internal function
inline std::string _Type2Str(jsScanHeadType type) {
  if (JS_SCAN_HEAD_JS50WX == type) {
    return "JS50WX";
  } else if (JS_SCAN_HEAD_JS50WSC == type) {
    return "JS50WSC";
  } else if (JS_SCAN_HEAD_JS50WSC == type) {
    return "JS50MX";
  } else if (JS_SCAN_HEAD_JS50MX == type) {
    return "JS50WSC";
  } else if (JS_SCAN_HEAD_JS50X6B20 == type) {
    return "JS50X6B20";
  } else if (JS_SCAN_HEAD_JS50X6B30 == type) {
    return "JS50X6B30";
  } else if (JS_SCAN_HEAD_JS50Z820 == type) {
    return "JS50Z820";
  } else if (JS_SCAN_HEAD_JS50Z830 == type) {
    return "JS50Z830";
  }

  return "JS50Invalid";
}

inline bool _IsLasHead(jsScanHeadType type) {
  if ((JS_SCAN_HEAD_JS50X6B20 == type) || (JS_SCAN_HEAD_JS50X6B30 == type) || (JS_SCAN_HEAD_JS50Z820 == type) ||
      (JS_SCAN_HEAD_JS50Z830 == type)) {
    return true;
  }

  return false;
}

/// Internal function
inline void _DummyLogger(jsError err, std::string msg) {
  // Suppress all messages
  (void)err;
  (void)msg;
}

/**
 * @brief A logging function may be provided to `jsSetupConfigParse` in order
 * to receive status and error messages. The function should be of the form:
 *
 *    void logger(jsError error, std::string message)
 *
 * While parsing the configuration file, this function will be periodically
 * called. If `JS_ERROR_NONE == error`, the message is a status message that
 * provides the current progress of parsing the file. If `error` is any other
 * value, the message is an error message that provides information as to what
 * failed.
 */
using Logger = void(jsError, std::string);

/**
 * @brief Parses a JSON configuration file from `jsSetup`, initializing and
 * the configuring the Scan System and all of its associated Scan Heads
 * according to the configuration file's contents.
 *
 * @param jsonConfig The complete JSON config to parse.
 * @param system Reference to uninitialized Scan System.
 * @param heads Reference to empty vector to be populated with Scan Heads.
 * @param logger Pointer to optional status & error logging function.
 * @return `0` on success, negative value mapping to `jsError` on error. On
 *         successful return, the `system` reference will be initialized
 *         and the `heads` vector will be populated with the Scan Heads in the
 *         configuration file, arranged by ID in ascending order. On error,
 *         the Scan System and Scan Heads should be assumed to be in an
 *         indeterminate state. It is recommended to free the Scan System and
 *         remedy the error in the configuration file, rather than continuing.
 */
inline jsError jsSetupConfigParse(const json& jsonConfig, jsScanSystem& system, std::vector<jsScanHead>& heads,
                                  Logger* logger = nullptr) {
  if (nullptr == logger) {
    // No logger provided; suppress messages
    logger = _DummyLogger;
  }

  try {
    // read a JSON file
    json jsubroot;
    int32_t r = 0;

    {
      const char* version;
      jsGetAPIVersion(&version);
      std::string version_str = "joescanapi " + std::string(version);
      logger(JS_ERROR_NONE, version_str);
    }

    std::string units_str = jsonConfig["Units"];
    jsUnits units = JS_UNITS_INVALID;
    if (0 == units_str.compare("Inches")) {
      units = JS_UNITS_INCHES;
    } else if (0 == units_str.compare("Millimeters")) {
      units = JS_UNITS_MILLIMETER;
    }

    logger(JS_ERROR_NONE, "Creating scan system");
    system = jsScanSystemCreate(units);
    if (0 > system) {
      jsError err = (jsError)system;
      logger(err, "jsScanSystemCreate failed");
      return err;
    }

    jsubroot = jsonConfig["ScanHeads"];
    for (json::iterator it = jsubroot.begin(); it != jsubroot.end(); ++it) {
      json jhead = it->get<json::object_t>();
      json jchild;
      uint32_t id = jhead["Id"];
      uint32_t serial = jhead["Serial"];
      std::string type_str = jhead["ProductType"];
      std::string cable_str = jhead["Orientation"];
      std::string head_str = type_str + " " + std::to_string(serial);

      logger(JS_ERROR_NONE, "Creating scan head " + head_str + ", ID " + std::to_string(id));
      jsScanHead head = jsScanSystemCreateScanHead(system, serial, id);
      if (0 > head) {
        jsError err = (jsError)head;
        logger(err, "jsScanSystemCreateScanHead failed");
        return err;
      }
      heads.push_back(head);

      jsScanHeadType type = _Str2Type(type_str);
      jsCableOrientation cable = _Str2Cable(cable_str);
      logger(JS_ERROR_NONE, head_str + " Orientation: " + cable_str);

      r = jsScanHeadSetCableOrientation(head, cable);
      if (0 > r) {
        logger((jsError)r, "jsScanHeadSetCableOrientation failed");
        return (jsError)r;
      }

      jchild = jhead["Exposure"];
      jsScanHeadConfiguration cfg;
      cfg.camera_exposure_time_min_us = jchild["MinExposureTimeUs"];
      cfg.camera_exposure_time_def_us = jchild["DefExposureTimeUs"];
      cfg.camera_exposure_time_max_us = jchild["MaxExposureTimeUs"];
      cfg.laser_on_time_min_us = jchild["MinLaserOnTimeUs"];
      cfg.laser_on_time_def_us = jchild["DefLaserOnTimeUs"];
      cfg.laser_on_time_max_us = jchild["MaxLaserOnTimeUs"];
      cfg.laser_detection_threshold = jchild["LaserDetectionThreshold"];
      cfg.saturation_threshold = jchild["SaturationThreshold"];
      cfg.saturation_percentage = jchild["SaturationPercentage"];
      logger(JS_ERROR_NONE, head_str + " Laser on time: " + std::to_string(cfg.laser_on_time_min_us) + "," +
                                std::to_string(cfg.laser_on_time_def_us) + "," +
                                std::to_string(cfg.laser_on_time_max_us));
      logger(JS_ERROR_NONE, head_str + " Camera exposure time: " + std::to_string(cfg.camera_exposure_time_min_us) +
                                "," + std::to_string(cfg.camera_exposure_time_def_us) + "," +
                                std::to_string(cfg.camera_exposure_time_max_us));
      logger(JS_ERROR_NONE, head_str + " Laser detection threshold: " + std::to_string(cfg.laser_detection_threshold));
      logger(JS_ERROR_NONE, head_str + " Saturation threshold: " + std::to_string(cfg.saturation_threshold));
      logger(JS_ERROR_NONE, head_str + " Saturation percentage: " + std::to_string(cfg.saturation_percentage));

      r = jsScanHeadSetConfiguration(head, &cfg);
      if (0 > r) {
        logger((jsError)r, "jsScanHeadSetConfiguration failed");
        return (jsError)r;
      }

      jchild = jhead["Alignments"];
      for (json::iterator itc = jchild.begin(); itc != jchild.end(); ++itc) {
        json jobj = itc->get<json::object_t>();

        std::string cam_str = jobj["Camera"];
        std::string las_str = jobj["Laser"];
        std::string pair_str = cam_str + " " + las_str;
        jsCamera camera = _Str2Cam(cam_str);
        jsLaser laser = _Str2Las(las_str);
        double roll = jobj["RollDeg"];
        double shiftx = jobj["ShiftX"];
        double shifty = jobj["ShiftY"];

        logger(JS_ERROR_NONE, head_str + " Alignment " + pair_str + ": roll " + std::to_string(roll) + ", shift X " +
                                  std::to_string(shiftx) + ", shift Y " + std::to_string(shifty));

        if (_IsLasHead(type)) {
          r = jsScanHeadSetAlignmentLaser(head, laser, roll, shiftx, shifty);
        } else {
          r = jsScanHeadSetAlignmentCamera(head, camera, roll, shiftx, shifty);
        }

        if (0 > r) {
          logger((jsError)r, "jsScanHeadSetAlignment failed");
          return (jsError)r;
        }
      }

      jchild = jhead["Windows"];
      for (json::iterator itc = jchild.begin(); itc != jchild.end(); ++itc) {
        json jobj = itc->get<json::object_t>();
        std::string cstr = jobj["Camera"];
        std::string lstr = jobj["Laser"];
        std::string pair_str = cstr + " " + lstr;
        std::string vertices_str = "";
        std::vector<jsCoordinate> vertices;
        jsCamera camera = _Str2Cam(cstr);
        jsLaser laser = _Str2Las(lstr);

        json jwin = jobj["Vertices"];

        if (jwin.empty()) {
          logger(JS_ERROR_NONE, head_str + " Window empty!");
        } else {
          for (json::iterator itv = jwin.begin(); itv != jwin.end(); ++itv) {
            json jvert = itv->get<json::object_t>();
            jsCoordinate v;
            v.x = jvert["X"];
            v.y = jvert["Y"];

            vertices.push_back(v);
            vertices_str += " (" + std::to_string(v.x) + "," + std::to_string(v.y) + ")";
          }
          logger(JS_ERROR_NONE, head_str + " Window" + vertices_str);
          if (_IsLasHead(type)) {
            r = jsScanHeadSetPolygonWindowLaser(head, laser, vertices.data(), vertices.size());
          } else {
            r = jsScanHeadSetPolygonWindowCamera(head, camera, vertices.data(), vertices.size());
          }

          if (0 > r) {
            logger((jsError)r, "jsScanHeadSetWindowRectangular failed");
            return (jsError)r;
          }
        }
      }

      jchild = jhead["ExclusionMasks"];
      for (json::iterator itc = jchild.begin(); itc != jchild.end(); ++itc) {
        json jobj = itc->get<json::object_t>();
        std::string cstr = jobj["Camera"];
        std::string lstr = jobj["Laser"];
        std::string pair_str = cstr + " " + lstr;
        jsCamera camera = _Str2Cam(cstr);
        jsLaser laser = _Str2Las(lstr);

        json jexc = jobj["ExcludedRegions"];

        if (jexc.empty()) {
          logger(JS_ERROR_NONE, head_str + " ExcludedRegions empty!");
        } else {
          jsExclusionMask* mask = new jsExclusionMask;
          memset(mask, 0, sizeof(jsExclusionMask));
          for (json::iterator itx = jexc.begin(); itx != jexc.end(); ++itx) {
            json jexr = itx->get<json::object_t>();
            uint32_t left = jexr["Left"];
            uint32_t top = jexr["Top"];
            uint32_t width = jexr["Width"];
            uint32_t height = jexr["Height"];

            logger(JS_ERROR_NONE, head_str + " Excluded Region " + pair_str + ": left " + std::to_string(left) +
                                      ", top " + std::to_string(top) + ", width " + std::to_string(width) +
                                      ", height " + std::to_string(height));

            for (uint32_t m = 0; m < height; m++) {
              for (uint32_t n = 0; n < width; n++) {
                mask->bitmap[m][n] = 1;
              }
            }
          }

          if (_IsLasHead(type)) {
            r = jsScanHeadSetExclusionMaskLaser(head, laser, mask);
          } else {
            r = jsScanHeadSetExclusionMaskCamera(head, camera, mask);
          }

          if (0 > r) {
            logger((jsError)r, "jsScanHeadSetExclusionMask failed");
            return (jsError)r;
          }

          delete mask;
        }
      }

      jchild = jhead["BrightnessCorrections"];
      for (json::iterator itc = jchild.begin(); itc != jchild.end(); ++itc) {
        json jobj = itc->get<json::object_t>();
        // TODO: Brightness Correction, waiting for support in jsSetup
      }
    }

    // Sort vector of heads by IDs, ascending order
    std::sort(heads.begin(), heads.end(), [](const jsScanHead& lhs, const jsScanHead& rhs) {
      const uint32_t lhs_id = jsScanHeadGetId(lhs);
      const uint32_t rhs_id = jsScanHeadGetId(rhs);
      return lhs_id < rhs_id;
    });

    // Phase entry information; to be used to build of Phase Table
    struct PhaseConfig {
      uint32_t id;
      uint32_t serial;
      uint32_t phase;
      jsScanHead head;
      jsScanHeadType type;
      jsCamera camera;
      jsLaser laser;
    };

    std::vector<PhaseConfig> phase_configs;
    jsubroot = jsonConfig["PhaseTableEntries"];
    for (json::iterator it = jsubroot.begin(); it != jsubroot.end(); ++it) {
      json child = it->get<json::object_t>();
      PhaseConfig pc;

      std::string cam_str = child["Camera"];
      std::string las_str = child["Laser"];

      pc.id = child["Id"];
      pc.phase = child["Phase"];
      pc.head = jsScanSystemGetScanHeadById(system, pc.id);
      pc.serial = jsScanHeadGetSerial(pc.head);
      pc.type = jsScanHeadGetType(pc.head);
      pc.camera = _Str2Cam(cam_str);
      pc.laser = _Str2Las(las_str);
      phase_configs.push_back(pc);
    }

    // Sort vector of phases by phase number, ascending order
    std::sort(phase_configs.begin(), phase_configs.end(),
              [](const struct PhaseConfig& lhs, const struct PhaseConfig& rhs) {
                return lhs.phase < rhs.phase;
              });

    int32_t current_phase = -1;
    for (auto& pc : phase_configs) {
      uint32_t phase = pc.phase;

      if ((int32_t)phase > current_phase) {
        logger(JS_ERROR_NONE, "Creating phase " + std::to_string(phase));
        r = jsScanSystemPhaseCreate(system);
        if (0 > r) {
          logger((jsError)r, "jsScanSytemCreatePhase failed");
          return (jsError)r;
        }

        std::string info =
            _Type2Str(pc.type) + " " + std::to_string(pc.serial) + " " + _Cam2Str(pc.camera) + " " + _Las2Str(pc.laser);

        logger(JS_ERROR_NONE, "Inserting " + info + " into phase " + std::to_string(phase));
        if (_IsLasHead(pc.type)) {
          r = jsScanSystemPhaseInsertLaser(system, pc.head, pc.laser);
        } else {
          r = jsScanSystemPhaseInsertCamera(system, pc.head, pc.camera);
        }

        if (0 > r) {
          logger((jsError)r, "jsScanSystemPhaseInsert failed");
          return (jsError)r;
        }
      }
    }
  } catch (const json::exception& e) {
    std::string what_str = e.what();
    std::string id_str = std::to_string(e.id);
    std::string err_str = "message: " + what_str + ", exception id: " + id_str;
    logger(JS_ERROR_INTERNAL, "JSON parse error, " + err_str);
    return JS_ERROR_INTERNAL;
  }

  return JS_ERROR_NONE;
}

}  // namespace joescan

#endif
