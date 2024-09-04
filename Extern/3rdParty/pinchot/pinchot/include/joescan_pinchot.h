/**
 * Copyright (c) JoeScan Inc. All Rights Reserved.
 *
 * Licensed under the BSD 3 Clause License. See LICENSE.txt in the project
 * root for license information.
 */

/**
 * @file joescan_pinchot.h
 * @author JoeScan
 * @brief This file contains the interface for the client software used to
 * control scanning for JoeScan products.
 */

#ifndef _JOESCAN_PINCHOT_H
#define _JOESCAN_PINCHOT_H

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma once

/**
 * @brief Macro function to check if `jsProfile` has been filled with data
 * from the scan head.
 *
 * @param profile The profile to check.
 * @return Boolean `true` if has valid data, `false` otherwise.
 */
#define jsProfileIsValid(profile) \
  (((profile).timestamp_ns != 0) && \
   ((profile).format != JS_DATA_FORMAT_INVALID)) ? \
  true : false

/**
 * @brief Macro function to check if `jsRawProfile` has been filled with data
 * from the scan head.
 *
 * @param profile The profile to check.
 * @return Boolean `true` if has valid data, `false` otherwise.
 */
#define jsRawProfileIsValid(profile) \
  (((profile).timestamp_ns != 0) && \
   ((profile).format != JS_DATA_FORMAT_INVALID)) ? \
  true : false

/**
 * @brief Opaque reference to an object in software used to manage a complete
 * system of scan heads.
 */
typedef int64_t jsScanSystem;

/**
 * @brief Opaque reference to an object in software that represents as single
 * physical scan head.
 */
typedef int64_t jsScanHead;

/**
 * @brief Constant values used with this API.
 */
enum jsConstants {
  /** @brief Maximum string length of JS-50 scan head. */
  JS_SCAN_HEAD_TYPE_STR_MAX_LEN = 32,
  /** @brief Maximum string length of client network interface name. */
  JS_CLIENT_NAME_STR_MAX_LEN = 128,
  /** @brief Maximum number of columns of scan data captured by the camera. */
  JS_SCAN_HEAD_DATA_COLUMNS_MAX_LEN = 1456,
  /** @brief Array length of data reserved for a profile. */
  JS_PROFILE_DATA_LEN = JS_SCAN_HEAD_DATA_COLUMNS_MAX_LEN,
  /** @brief Array length of data reserved for a raw profile. */
  JS_RAW_PROFILE_DATA_LEN = JS_SCAN_HEAD_DATA_COLUMNS_MAX_LEN,
  /** @brief Maximum number of columns in an image taken from the scan head. */
  JS_CAMERA_IMAGE_DATA_MAX_WIDTH = JS_SCAN_HEAD_DATA_COLUMNS_MAX_LEN,
  /** @brief Maximum number of rows in an image taken from the scan head. */
  JS_CAMERA_IMAGE_DATA_MAX_HEIGHT = 1088,
  /** @brief Array length of data reserved for an image. */
  JS_CAMERA_IMAGE_DATA_LEN =
    JS_CAMERA_IMAGE_DATA_MAX_HEIGHT * JS_CAMERA_IMAGE_DATA_MAX_WIDTH,
  /**
   * @brief Value that `x` and `y` will be assigned in `jsProfileData` if the
   * point is invalid.
   */
  JS_PROFILE_DATA_INVALID_XY = INT_MIN,
  /**
   * @brief Value that `brightness` will be assigned in `jsProfileData` if
   * the measurement is invalid.
   */
  JS_PROFILE_DATA_INVALID_BRIGHTNESS = 0,
  /**
   * @brief The maximum number of profiles that can be read from a given
   * scan head with one API call.
   */
  JS_SCAN_HEAD_PROFILES_MAX = 1000,
};

/**
 * @brief Enumerated value for possible errors returned from API functions.
 *
 * @note These values can be converted to a string value by using the
 * `jsGetError` function.
 */
enum jsError {
  /** @brief No error. */
  JS_ERROR_NONE = 0,
  /** @brief Error resulted from internal code. */
  JS_ERROR_INTERNAL = -1,
  /** @brief Error resulted from `NULL` value passed in as argument. */
  JS_ERROR_NULL_ARGUMENT = -2,
  /** @brief Error resulted from incorrect or out of range argument value. */
  JS_ERROR_INVALID_ARGUMENT = -3,
  /** @brief Error resulted from the system not being in a connected state. */
  JS_ERROR_NOT_CONNECTED = -4,
  /** @brief Error resulted from the system being in a connected state. */
  JS_ERROR_CONNECTED = -5,
  /** @brief Error occurred from the system not being in a scanning state. */
  JS_ERROR_NOT_SCANNING = -6,
  /** @brief Error occurred from the system being in a scanning state. */
  JS_ERROR_SCANNING = -7,
  /** @brief Error occurred from a version compatibility issue between the scan
     head(s) and the API. */
  JS_ERROR_VERSION_COMPATIBILITY = -8,
  /** @brief Error occurred trying to add or create something that already has
      been done and would result in a duplicate. */
  JS_ERROR_ALREADY_EXISTS = -9,
  /** @brief Error occurred trying to add or create something, but due to
      constraints this can not be done. */
  JS_ERROR_NO_MORE_ROOM = -10,
  /** @brief Error occurred with networking interface or communication. */
  JS_ERROR_NETWORK = -11,
  /** @brief Error occurred with scan head not found on the network. */
  JS_ERROR_NOT_DISCOVERED = -12,
  /** @brief Error from wrong function call, use `*Camera` variant function. */
  JS_ERROR_USE_CAMERA_FUNCTION = -13,
  /** @brief Error from wrong function call, use `*Laser` variant function. */
  JS_ERROR_USE_LASER_FUNCTION = -14,
  /** @brief Error from action not compatible with frame scanning. */
  JS_ERROR_FRAME_SCANNING = -15,
  /** @brief Error from action only compatible with frame scanning. */
  JS_ERROR_NOT_FRAME_SCANNING = -16,
  /** @brief Configured phase table is not compatible with frame scanning. */
  JS_ERROR_FRAME_SCANNING_INVALID_PHASE_TABLE = -17,
  /** @brief Error occurred for an unknown reason; this should never happen. */
  JS_ERROR_UNKNOWN = -18,

  JS_ERROR_FORCE_INT32_SIZE = INT32_MAX,
};

/**
 * @brief The units that a given scan system and all associated scan heads will
 * use for configuration and returned data.
 */
typedef enum {
  JS_UNITS_INVALID = 0,
  JS_UNITS_INCHES = 1,
  JS_UNITS_MILLIMETER = 2,

  JS_UNITS_FORCE_INT32_SIZE = INT32_MAX,
} jsUnits;

/**
 * @brief The camera orientation for a given scan head; the orientation
 * selected influences the coordinate system of returned data.
 */
typedef enum {
  JS_CABLE_ORIENTATION_INVALID = 0,
  JS_CABLE_ORIENTATION_DOWNSTREAM = 1,
  JS_CABLE_ORIENTATION_UPSTREAM = 2,

  JS_CABLE_ORIENTATION_FORCE_INT32_SIZE = INT32_MAX,
} jsCableOrientation;

/**
 * @brief Enumerated value identifying the scan head type.
 */
typedef enum {
  JS_SCAN_HEAD_INVALID_TYPE = 0,
  JS_SCAN_HEAD_JS50WX = 1,
  JS_SCAN_HEAD_JS50WSC = 2,
  JS_SCAN_HEAD_JS50X6B20 = 3,
  JS_SCAN_HEAD_JS50X6B30 = 4,
  JS_SCAN_HEAD_JS50MX = 5,
  JS_SCAN_HEAD_JS50Z820 = 6,
  JS_SCAN_HEAD_JS50Z830 = 7,

  JS_SCAN_HEAD_TYPE_FORCE_INT32_SIZE = INT32_MAX,
} jsScanHeadType;

/**
 * @brief Data type for identifying a camera on the scan head.
 */
typedef enum {
  JS_CAMERA_INVALID = 0,
  JS_CAMERA_A = 1,
  JS_CAMERA_B,
  JS_CAMERA_MAX,

  JS_CAMERA_FORCE_INT32_SIZE = INT32_MAX,
} jsCamera;

/**
 * @brief Data type for identifying a laser on the scan head.
 */
typedef enum {
  JS_LASER_INVALID = 0,
  JS_LASER_1 = 1,
  JS_LASER_2,
  JS_LASER_3,
  JS_LASER_4,
  JS_LASER_5,
  JS_LASER_6,
  JS_LASER_7,
  JS_LASER_8,
  JS_LASER_MAX,

  JS_LASER_FORCE_INT32_SIZE = INT32_MAX,
} jsLaser;

/**
 * @brief Data type for identifying an encoder on the scan head.
 */
typedef enum {
  JS_ENCODER_MAIN = 0,
  JS_ENCODER_AUX_1,
  JS_ENCODER_AUX_2,
  JS_ENCODER_MAX,

  JS_ENCODER_FORCE_INT32_SIZE = INT32_MAX,
} jsEncoder;

typedef enum {
  /** @brief ScanSync encoder A+/A- input connection is faulty. */
  JS_PROFILE_FLAG_ENCODER_MAIN_FAULT_A = 1 << 0,
  /** @brief ScanSync encoder B+/B- input connection is faulty. */
  JS_PROFILE_FLAG_ENCODER_MAIN_FAULT_B = 1 << 1,
  /** @brief ScanSync aux Y+/Y- input connection is faulty. */
  JS_PROFILE_FLAG_ENCODER_MAIN_FAULT_Y = 1 << 2,
  /** @brief ScanSync index Z+/Z- input connection is faulty. */
  JS_PROFILE_FLAG_ENCODER_MAIN_FAULT_Z = 1 << 3,
  /** @brief ScanSync encoder data rate exceeds hardware capabilities. */
  JS_PROFILE_FLAG_ENCODER_MAIN_OVERRUN = 1 << 4,
  /** @brief ScanSync termination resistor pairs installed. */
  JS_PROFILE_FLAG_ENCODER_MAIN_TERMINATION_ENABLE = 1 << 5,
  /** @brief ScanSync index Z input is logic high. */
  JS_PROFILE_FLAG_ENCODER_MAIN_INDEX_Z = 1 << 6,
  /** @brief ScanSync sync input is logic high. */
  JS_PROFILE_FLAG_ENCODER_MAIN_SYNC = 1 << 7,

  JS_PROFILE_FLAGS_FORCE_INT32_SIZE = INT32_MAX,
} jsProfileFlags;

/**
 * @brief Enumerated value representing the types of data and the formats it
 * can take. For full resolution data formats, every data entry will be filled
 * within the returned profile's `data` array. Selecting half or quarter
 * resolution will result in every other or every fourth entry in the `data`
 * array to be filled respectively.
 */
typedef enum {
  JS_DATA_FORMAT_INVALID = 0,
  // Geometry and laser line brightness at combinations of full, 1/2, and 1/4
  // resolution.
  JS_DATA_FORMAT_XY_BRIGHTNESS_FULL,
  JS_DATA_FORMAT_XY_BRIGHTNESS_HALF,
  JS_DATA_FORMAT_XY_BRIGHTNESS_QUARTER,

  // Geometry at full, 1/2 and 1/4 resolution, no laser line brightness.
  JS_DATA_FORMAT_XY_FULL,
  JS_DATA_FORMAT_XY_HALF,
  JS_DATA_FORMAT_XY_QUARTER,

  JS_DATA_FORMAT_FORCE_INT32_SIZE = INT32_MAX,
} jsDataFormat;

/**
 * @brief Data type for setting fixed camera & laser exposure or to use the
 * auto exposure algorithm when obtaining diagnostic profiles and images.
 */
typedef enum {
  JS_DIAGNOSTIC_MODE_INVALID = 0,
  JS_DIAGNOSTIC_FIXED_EXPOSURE,
  JS_DIAGNOSTIC_AUTO_EXPOSURE,

  JS_DIAGNOSTIC_FORCE_INT32_SIZE = INT32_MAX,
} jsDiagnosticMode;

#pragma pack(push, 1)

typedef enum {
  JS_SCAN_HEAD_STATE_INVALID = 0,
  JS_SCAN_HEAD_STATE_IDLE = 1,
  JS_SCAN_HEAD_STATE_CONNECTED = 2,
  JS_SCAN_HEAD_STATE_SCANNING = 3,

  JS_SCAN_HEAD_STATE_FORCE_INT32_SIZE = INT32_MAX,
} jsScanHeadState;

/**
 * @brief Structure used to provide information as to scan heads discovered on
 * the network.
 */
typedef struct {
  /** @brief Serial number of scan head. */
  uint32_t serial_number;
  /** @brief Enumerated type of scan head. */
  jsScanHeadType type;
  /** @brief Null terminated string of scan head type. */
  char type_str[JS_SCAN_HEAD_TYPE_STR_MAX_LEN];
  /** @brief Firmware major version number of the scan head. */
  uint32_t firmware_version_major;
  /** @brief Firmware minor version number of the scan head. */
  uint32_t firmware_version_minor;
  /** @brief Firmware patch version number of the scan head. */
  uint32_t firmware_version_patch;
  /** @brief IP address of scan head. */
  uint32_t ip_addr;
  /** @brief Name of the client interface scan head was discovered on. */
  char client_name_str[JS_CLIENT_NAME_STR_MAX_LEN];
  /** @brief IP address of the client interface scan head was discovered on. */
  uint32_t client_ip_addr;
  /** @brief Netmask of the client interface scan head was discovered on. */
  uint32_t client_netmask;
  /** @brief Link speed in megabits per second between client and scan head. */
  uint32_t link_speed_mbps;
  /** @brief Current state of the scan head. */
  jsScanHeadState state;
} jsDiscovered;

/**
 * @brief Structure used to communicate the various capabilities and limits of
 * a given scan head type.
 */
typedef struct {
  /** @brief Number of bits used for a `brightness` value in `jsProfileData`. */
  uint32_t camera_brightness_bit_depth;
  /** @brief Maximum image height camera supports. */
  uint32_t max_camera_image_height;
  /** @brief Maximum image width camera supports. */
  uint32_t max_camera_image_width;
  /** @brief The smallest scan period in microseconds supported by product. */
  uint32_t min_scan_period_us;
  /** @brief The largest scan period in microseconds supported by product. */
  uint32_t max_scan_period_us;
  /** @brief The number of cameras supported by product. */
  uint32_t num_cameras;
  /** @brief The number of encoders supported by product. */
  uint32_t num_encoders;
  /** @brief The number of lasers supported by product. */
  uint32_t num_lasers;
} jsScanHeadCapabilities;

/**
 * @brief Structure used to configure a scan head's operating parameters.
 */
typedef struct {
  /** @deprecated Will be removed in a future release. */
  uint32_t camera_exposure_time_min_us;
  /** @deprecated Will be removed in a future release. */
  uint32_t camera_exposure_time_max_us;
  /** @deprecated Will be removed in a future release. */
  uint32_t camera_exposure_time_def_us;
  /**
   * @brief Sets the minimum microseconds time value for the laser on
   * algorithm. This value should be within the range of 15 to 650000
   * microseconds.
   *
   * @note To disable the laser on algorithm, set `laser_on_time_min_us`,
   * `laser_on_time_max_us`, and `laser_on_time_def_us` to the same value.
   */
  uint32_t laser_on_time_min_us;
  /**
   * @brief Sets the maximum microseconds time value for the laser on
   * algorithm. This value should be within the range of 15 to 650000
   * microseconds.
   *
   * @note To disable the laser on algorithm, set `laser_on_time_min_us`,
   * `laser_on_time_max_us`, and `laser_on_time_def_us` to the same value.
   */
  uint32_t laser_on_time_max_us;
  /**
   * @brief Sets the default microseconds time value for the laser on
   * algorithm. This value should be within the range of 15 to 650000
   * microseconds.
   *
   * @note To disable the laser on algorithm, set `laser_on_time_min_us`,
   * `laser_on_time_max_us`, and `laser_on_time_def_us` to the same value.
   */
  uint32_t laser_on_time_def_us;
  /**
   * @brief The minimum brightness a data point must have to be considered a
   * valid data point. Value must be between `0` and `1023`.
   */
  uint32_t laser_detection_threshold;
  /**
   * @brief Set how bright a data point must be to be considered saturated.
   * Value must be between `0` and `1023`.
   */
  uint32_t saturation_threshold;
  /**
   * @brief Set the maximum percentage of the pixels in a scan that are allowed
   * to be brighter than the saturation threshold. Value must be between `0`
   * and `100`.
   */
  uint32_t saturation_percentage;
} jsScanHeadConfiguration;

/**
 * @brief Structure used to define which pixels in the camera to exclude from
 * scan data.
 */
typedef struct {
  /**
   * @brief A 2D array with each entry representing one pixel in view of the
   * camera. Setting to a non-zero value will block that pixel from being used
   * when scanning; ensuring no scan data being generated at that point.
   */
  uint8_t bitmap[JS_CAMERA_IMAGE_DATA_MAX_HEIGHT]
                [JS_CAMERA_IMAGE_DATA_MAX_WIDTH];
} jsExclusionMask;

/**
 * @brief Structure used to adjust and scale the brightness values returned
 * with scan data. The brightness is corrected in the following manner:
 *
 *   new_brightness[col] = original_brightness[col] *
 *                         scale_factors[col] + offset
 */
typedef struct {
  /** @brief Value to offset brightness for all scan data columns. */
  uint8_t offset;
  /**
   * @brief Value to scale brightness for a particular column of scan data.
   *
   * @note The array entries correspond to the same position of profile data
   * points as ordered in the `jsRawProfile` struct.
   */
  float scale_factors[JS_SCAN_HEAD_DATA_COLUMNS_MAX_LEN];
} jsBrightnessCorrection_BETA;

/**
 * @brief Structure used to hold information pertaining to the scan head.
 */
typedef struct {
  /** @brief System global time in nanoseconds. */
  uint64_t global_time_ns;
  /** @brief The current encoder positions. */
  int64_t encoder_values[JS_ENCODER_MAX];
  /** @brief The number of encoder values available. */
  uint32_t num_encoder_values;
  /** @brief Total number of pixels seen by the camera A's scan window. */
  int32_t camera_a_pixels_in_window;
  /** @brief Total number of pixels seen by the camera B's scan window. */
  int32_t camera_b_pixels_in_window;
  /** @brief Current temperature in Celsius reported by camera A. */
  int32_t camera_a_temp;
  /** @brief Current temperature in Celsius reported by camera B. */
  int32_t camera_b_temp;
  /** @brief Total number of profiles sent during the last scan period. */
  uint32_t num_profiles_sent;
} jsScanHeadStatus;

/**
 * @brief A spatial coordinate point in scan system units.
 */
typedef struct {
  /**
   * @brief The X coordinate in scan system units.
   */
  double x;
  /**
   * @brief The Y coordinate in scan system units.
   */
  double y;
} jsCoordinate;

/**
 * @brief A data point within a returned profile's data.
 */
typedef struct {
  /**
   * @brief The X coordinate in 1/1000 scan system units.
   * @note If invalid, will be set to `JS_PROFILE_DATA_INVALID_XY`.
   */
  int32_t x;
  /**
   * @brief The Y coordinate in 1/1000 scan system units.
   * @note If invalid, will be set to `JS_PROFILE_DATA_INVALID_XY`.
   */
  int32_t y;
  /**
   * @brief Measured brightness at given point.
   * @note If invalid, will be set to `JS_PROFILE_DATA_INVALID_BRIGHTNESS`.
   */
  int32_t brightness;
} jsProfileData;

/**
 * @brief Scan data is returned from the scan head through profiles; each
 * profile returning a single scan line at a given moment in time.
 */
typedef struct {
  /** @brief The Id of the scan head that the profile originates from. */
  uint32_t scan_head_id;
  /** @brief The camera used for the profile. */
  jsCamera camera;
  /** @brief The laser used for the profile. */
  jsLaser laser;
  /** @brief Time of the scan head in nanoseconds when profile was taken. */
  uint64_t timestamp_ns;
  /** @brief Bitmask of flags listed in `enum jsProfileFlags`. */
  uint32_t flags;
  /** @brief Monotonically increasing count of profiles generated by camera. */
  uint32_t sequence_number;
  /** @brief Array holding current encoder values. */
  int64_t encoder_values[JS_ENCODER_MAX];
  /** @brief Number of encoder values in this profile. */
  uint32_t num_encoder_values;
  /** @brief Time in microseconds for the laser emitting. */
  uint32_t laser_on_time_us;
  /**
   * @brief The format of the data for the given `jsProfile`.
   */
  jsDataFormat format;
  /** @deprecated Will be removed in a future release. */
  uint32_t packets_received;
  /** @deprecated Will be removed in a future release. */
  uint32_t packets_expected;
  /**
   * @brief The total number of valid scan line measurement points for this
   * profile held in the `data` array.
   */
  uint32_t data_len;
  /** @brief Reserved for future use. */
  uint64_t reserved_0;
  /** @brief Reserved for future use. */
  uint64_t reserved_1;
  /** @brief Reserved for future use. */
  uint64_t reserved_2;
  /** @brief Reserved for future use. */
  uint64_t reserved_3;
  /** @brief Reserved for future use. */
  uint64_t reserved_4;
  /** @brief Reserved for future use. */
  uint64_t reserved_5;
  /** @brief An array of scan line data associated with this profile. */
  jsProfileData data[JS_PROFILE_DATA_LEN];
} jsProfile;

/**
 * @brief A Raw Profile is the most basic type of profile returned back from
 * a scan head. The data is left unprocessed with the contents being dependent
 * on the `jsDataFormat` that the scan head is configured for.
 *
 * @note It is highly recommended to use `jsProfile` rather than `jsRawProfile`
 * as the former is far more simplified and user friendly than the later which
 * presents low level API implementation details.
 */
typedef struct {
  /** @brief The Id of the scan head that the profile originates from. */
  uint32_t scan_head_id;
  /** @brief The camera used for the profile. */
  jsCamera camera;
  /** @brief The laser used for the profile. */
  jsLaser laser;
  /** @brief Time of the scan head in nanoseconds when profile was taken. */
  uint64_t timestamp_ns;
  /** @brief Bitmask of flags listed in `enum jsProfileFlags`. */
  uint32_t flags;
  /** @brief Monotonically increasing count of profiles generated by camera. */
  uint32_t sequence_number;
  /** @brief Array holding current encoder values. */
  int64_t encoder_values[JS_ENCODER_MAX];
  /** @brief Number of encoder values in this profile. */
  uint32_t num_encoder_values;
  /** @brief Time in microseconds for the laser emitting. */
  uint32_t laser_on_time_us;
  /**
   * @brief The arrangement profile data contained in the `data` array. For raw
   * profile data, the data will be filled in at most according to the
   * resolution specified in the `jsDataFormat` type. For full resolution, the
   * entire scan line will be sampled. For half resolution, half of the scan
   * line will be sampled and `data` array will be populated at every other
   * entry. Similarly for quarter resolution, every fourth entry will be used
   * for scan data. Comparison with `JS_PROFILE_DATA_INVALID_XY` for X/Y values
   * and `JS_PROFILE_DATA_INVALID_BRIGHTNESS` can be used to determine if
   * data is set or not.
   */
  jsDataFormat format;
  /** @deprecated Will be removed in a future release. */
  uint32_t packets_received;
  /** @deprecated Will be removed in a future release. */
  uint32_t packets_expected;
  /**
   * @brief The total length of profile data held in the `data` array. This
   * value will be less than or equal to `JS_RAW_PROFILE_DATA_LEN` and should
   * be used for iterating over the array.
   */
  uint32_t data_len;
  /**
   * @brief Number of `brightness` values in the `data` array that are valid.
   * Invalid `brightness` will be set to `JS_PROFILE_DATA_INVALID_BRIGHTNESS`.
   */
  uint32_t data_valid_brightness;
  /**
   * @brief Number of `x` and `y` values in the `data` array that are valid.
   * Invalid `x` and `y` will have both set to `JS_PROFILE_DATA_INVALID_XY`.
   */
  uint32_t data_valid_xy;
  /** @brief Reserved for future use. */
  uint64_t reserved_0;
  /** @brief Reserved for future use. */
  uint64_t reserved_1;
  /** @brief Reserved for future use. */
  uint64_t reserved_2;
  /** @brief Reserved for future use. */
  uint64_t reserved_3;
  /** @brief Reserved for future use. */
  uint64_t reserved_4;
  /** @brief Reserved for future use. */
  uint64_t reserved_5;
  /** @brief An array of scan line data associated with this profile. */
  jsProfileData data[JS_RAW_PROFILE_DATA_LEN];
} jsRawProfile;

/**
 * @brief This structure is used to return a greyscale image capture from the
 * scan head.
 */
typedef struct {
  /** @brief The Id of the scan head that the image originates from. */
  uint32_t scan_head_id;
  /** @brief The camera used to capture the image. */
  jsCamera camera;
  /** @brief The laser used during the image capture. */
  jsLaser laser;
  /** @brief Time of the scan head in nanoseconds when image was taken. */
  uint64_t timestamp_ns;
  /** @brief Array holding current encoder values. */
  uint64_t encoder_values[JS_ENCODER_MAX];
  /** @brief Number of encoder values in this profile. */
  uint32_t num_encoder_values;
  /** @brief Time in microseconds for the camera's exposure. */
  uint32_t camera_exposure_time_us;
  /** @brief Time in microseconds the laser is emitting. */
  uint32_t laser_on_time_us;

  /** @brief The overall height of the image in pixels. */
  uint32_t image_height;
  /** @brief The overall width of the image in pixels. */
  uint32_t image_width;

  /** @brief An array of pixel data representing the image. */
  uint8_t data[JS_CAMERA_IMAGE_DATA_LEN];
} jsCameraImage;

#pragma pack(pop)

#ifndef NO_PINCHOT_INTERFACE

// Macros for setting the visiblity of functions within the library.
#if defined _WIN32 || defined __CYGWIN__
  #ifdef WIN_EXPORT
    #ifdef __GNUC__
      #define EXPORTED __attribute__((dllexport))
    #else
      #define EXPORTED __declspec(dllexport)
    #endif
  #else
    #ifdef __GNUC__
      #define EXPORTED __attribute__((dllexport))
    #else
      #define EXPORTED __declspec(dllexport)
    #endif
  #endif
  #define NOT_EXPORTED
  #define PRE __cdecl
  #define POST
#else
  #if __GNUC__ >= 4
    #define EXPORTED __attribute__((visibility("default")))
    #define NOT_EXPORTED __attribute__((visibility("hidden")))
  #else
    #define EXPORTED
    #define NOT_EXPORTED
  #endif
  #define PRE
  #define POST __attribute__((sysv_abi))
#endif

/**
 * @brief Obtains the semantic version of the client API presented in this
 * header. The version string will be of the form `vX.Y.Z`, where `X` is the
 * major version number, `Y` is the minor version number, and `Z` is the patch
 * number. In some cases, additional information may be appended to the version
 * string, separated by hyphens.
 *
 * @param version_str Address to be updated with API version string.
 */
EXPORTED void PRE jsGetAPIVersion(
  const char **version_str) POST;

/**
 * @brief Obtains the semantic version of the client API presented as unsigned
 * integer values.
 *
 * @param major The major version number.
 * @param minor The minor version number.
 * @param patch The patch version number.
 */
EXPORTED void PRE jsGetAPISemanticVersion(
  uint32_t *major,
  uint32_t *minor,
  uint32_t *patch) POST;

/**
 * @brief Converts a `jsError` error value returned from an API function call
 * to a string value.
 *
 * @param return_code The `jsError` value returned from API call.
 * @param error_str Address to be updated with error string.
 */
EXPORTED void PRE jsGetError(
  int32_t return_code,
  const char **error_str) POST;

/**
 * @brief Initializes a given `jsProfile` to a known invalid state to indicate
 * it has not been populated with data from the scan head.
 *
 * @param profile Reference to `jsProfile` to initialize.
 */
EXPORTED void PRE jsProfileInit(
  jsProfile *profile) POST;

/**
 * @brief Initializes a given `jsRawProfile` to a known invalid state to
 * indicate it has not been populated with data from the scan head.
 *
 * @param profile Reference to `jsRawProfile` to initialize.
 */
EXPORTED void PRE jsRawProfileInit(
  jsRawProfile *profile) POST;

/**
 * @brief Performs a remote soft power cycle of a scan head.
 *
 * @note This function will only work with scan heads running v16.x.x firmware
 * or higher.
 *
 * @note Extreme care should be used when using this function. The scan head
 * being power cycled should NOT be connected or scanning. It is recommended
 * to have the entire scan system in a disconnected state.
 *
 * @note After this function successfully completes, it will take several
 * seconds before the scan head will appear on the network and be available for
 * use. On average, the scan head will take 30 seconds to reboot.
 *
 * @param serial_number The serial number of the scan head to power cycle.
 * @return `0` on success, negative value `jsError` on error.
 */
EXPORTED
int32_t jsPowerCycleScanHead(
  uint32_t serial_number) POST;

/**
 * @brief Creates a `jsScanSystem` used to manage and coordinate `jsScanHead`
 * objects.
 *
 * @param units The units the scan system and all scan heads will use.
 * @return Positive valued token on success, negative value mapping to `jsError`
 * on error.
 */
EXPORTED jsScanSystem PRE jsScanSystemCreate(
  jsUnits units) POST;

/**
 * @brief Frees a `jsScanSystem` and all resources associated with it. In
 * particular, this will free all `jsScanHead` objects created by this
 * object.
 *
 * @param scan_system Reference to system that will be freed.
 */
EXPORTED void PRE jsScanSystemFree(
  jsScanSystem scan_system) POST;

/**
 * @brief Performs a network discovery to determine what scan heads are on the
 * network.
 *
 * @param scan_system The scan system to perform discovery.
 * @return The total number of discovered scan heads on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int PRE jsScanSystemDiscover(
  jsScanSystem scan_system) POST;

/**
 * @brief Obtains a list of all of the scan heads discovered on the network.
 *
 * @param scan_system The scan system that previously performed discovery.
 * @param results  Pointer to memory to store discover data. Note, the memory
 * pointed to by `results` must be at least `sizeof(jsDiscovered) * max_results`
 * in total number of bytes available.
 * @param max_results The maximum number of discovered results to read.
 * @return The total number of discovered scan heads on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int PRE jsScanSystemGetDiscovered(
  jsScanSystem scan_system,
  jsDiscovered *results,
  uint32_t max_results) POST;

/**
 * @brief Gets the most recent encoder value reported by the ScanSync.
 *
 * @note The ScanSync provides encoder updates at 1ms intervals.
 *
 * @param scan_system Reference to the scan system.
 * @param encoder The selected encoder to obtain value from.
 * @param value Pointer to be updated with encoder value.
 *
 * @return `0` on success, negative value `jsError` on error.
 */
EXPORTED int PRE jsScanSystemGetEncoder(
  jsScanSystem scan_system,
  jsEncoder encoder,
  int64_t *value) POST;

/**
 * @brief Creates a `jsScanHead` object representing a physical scan head
 * within the system.
 *
 * @note This function can only be called when the scan system is disconnected.
 * Once `jsScanSystemConnect()` is called, `jsScanSystemDisconnect()` must be
 * called if new scan heads are desired to be created.
 *
 * @param scan_system Reference to system that will own the scan head.
 * @param serial The serial number of the physical scan head.
 * @param id A user defined numerically unique id to assign to this scan head.
 * @return Positive valued token on success, negative value mapping to `jsError`
 * on error.
 */
EXPORTED jsScanHead PRE jsScanSystemCreateScanHead(
  jsScanSystem scan_system,
  uint32_t serial,
  uint32_t id) POST;

/**
 * @brief Obtains a reference to an existing `jsScanHead` object.
 *
 * @param scan_system Reference to system that owns the scan head.
 * @param id The numeric ID of the `jsScanHead` object.
 * @return Positive valued token on success, negative value mapping to `jsError`
 * on error.
 */
EXPORTED jsScanHead PRE jsScanSystemGetScanHeadById(
  jsScanSystem scan_system,
  uint32_t id) POST;

/**
 * @brief Obtains a reference to an existing `jsScanHead` object.
 *
 * @param scan_system Reference to system that owns the scan head.
 * @param serial The serial number of the physical scan head.
 * @return Positive valued token on success, negative value mapping to `jsError`
 * on error.
 */
EXPORTED jsScanHead PRE jsScanSystemGetScanHeadBySerial(
  jsScanSystem scan_system,
  uint32_t serial) POST;

/**
 * @brief Returns the total number of scan heads within a given system. This
 * should equal the number of times `jsScanSystemCreateScanHead()` was
 * successfully called with a new serial number.
 *
 * @param scan_system Reference to system that owns the scan heads.
 * @return The number of scan heads on success, negative value mapping to
 * `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemGetNumberScanHeads(
  jsScanSystem scan_system) POST;

/**
 * @brief Attempts to connect to all scan heads within the system.
 *
 * @param scan_system Reference to system owning scan heads to connect to.
 * @param timeout_s TCP timeout for all managed scan heads in seconds.
 * @return The total number of connected scan heads on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemConnect(
  jsScanSystem scan_system,
  int32_t timeout_s) POST;

/**
 * @brief Disconnects all scan heads from a given system.
 *
 * @param scan_system Reference to system of scan heads.
 * @return `0` on success, negative value `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemDisconnect(
  jsScanSystem scan_system) POST;

/**
 * @brief Gets connected state for a scan system.
 *
 * @note A scan system is said to be connected if all of the scan heads
 * associated with it are connected.
 *
 * @param scan_system Reference to system of scan heads.
 * @return Boolean `true` if connected, `false` if disconnected.
 */
EXPORTED bool PRE jsScanSystemIsConnected(
  jsScanSystem scan_system) POST;

/**
 * @brief Clears all phases created for a given scan system previously made
 * through `jsScanSystemPhaseCreate`.
 *
 * @param scan_system Reference to the scan system.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemPhaseClearAll(
  jsScanSystem scan_system) POST;

/**
 * @brief Creates a new phase entry for the scan system. If no phases have been
 * created for the scan system it will create the first phase entry. If entries
 * have been created already through previous calls to `jsScanSystemPhaseCreate`
 * then the new phase will be placed after the last created phase.
 *
 * @param scan_system Reference to the scan system.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemPhaseCreate(
  jsScanSystem scan_system) POST;

/**
 * @brief Inserts a scan head and it's camera into a given phase entry.
 * Multiple scan heads may be inserted into a given phase entry. However, the
 * the total phase time for this paticular entry will be constrained by the
 * scan head that requires the most time to complete it's scan.
 *
 * @note This function should be used with scan heads that are camera driven.
 *
 * @param scan_system Reference to the scan system.
 * @param scan_head Reference to the scan head.
 * @param camera The camera of the scan head to scan with during phase entry.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemPhaseInsertCamera(
  jsScanSystem scan_system,
  jsScanHead scan_head,
  jsCamera camera) POST;

/**
 * @brief Inserts a scan head and it's camera into a given phase entry.
 * Multiple scan heads may be inserted into a given phase entry. However, the
 * the total phase time for this paticular entry will be constrained by the
 * scan head that requires the most time to complete it's scan.
 *
 * @note This function should be used with scan heads that are laser driven.
 *
 * @param scan_system Reference to the scan system.
 * @param scan_head Reference to the scan head.
 * @param laser The camera of the scan head to scan with during phase entry.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemPhaseInsertLaser(
  jsScanSystem scan_system,
  jsScanHead scan_head,
  jsLaser laser) POST;

/**
 * @brief Inserts a scan head and it's camera into a given phase entry.
 * Multiple scan heads may be inserted into a given phase entry. However, the
 * the total phase time for this paticular entry will be constrained by the
 * scan head that requires the most time to complete it's scan.
 *
 * @note This function will use the `jsScanHeadConfiguration` provided as a
 * function argument when configuring this particular camera for scanning
 * rather than that specified in `jsScanHeadSetConfiguration`.
 *
 * @note Only the `laser_on_time_max_us`, `laser_on_time_def_us`, and
 * `laser_on_time_min_us` fields from `jsScanHeadSetConfiguration`
 * are currently used for per phase configuration.
 *
 * @note This function should be used with scan heads that are camera driven.
 *
 * @param scan_system Reference to the scan system.
 * @param scan_head Reference to the scan head.
 * @param cfg Configuration to be applied to the camera.
 * @param camera The camera of the scan head to scan with during phase entry.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemPhaseInsertConfigurationCamera(
  jsScanSystem scan_system,
  jsScanHead scan_head,
  jsScanHeadConfiguration *cfg,
  jsCamera camera) POST;

/**
 * @brief Inserts a scan head and it's laser into a given phase entry.
 * Multiple scan heads may be inserted into a given phase entry. However, the
 * the total phase time for this paticular entry will be constrained by the
 * scan head that requires the most time to complete it's scan.
 *
 * @note This function will use the `jsScanHeadConfiguration` provided as a
 * function argument when configuring this particular camera for scanning
 * rather than that specified in `jsScanHeadSetConfiguration`.
 *
 * @note Only the `laser_on_time_max_us`, `laser_on_time_def_us`, and
 * `laser_on_time_min_us` fields from `jsScanHeadSetConfiguration` are
 * currently used for per phase configuration.
 *
 * @note This function should be used with scan heads that are laser driven.
 *
 * @param scan_system Reference to the scan system.
 * @param scan_head Reference to the scan head.
 * @param cfg Configuration to be applied to the laser.
 * @param laser The camera of the scan head to scan with during phase entry.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemPhaseInsertConfigurationLaser(
  jsScanSystem scan_system,
  jsScanHead scan_head,
  jsScanHeadConfiguration *cfg,
  jsLaser laser) POST;

/**
 * @deprecated Use `jsScanSystemPhaseInsertConfigurationCamera`.
 *
 * @return `JS_ERROR_INVALID_ARGUMENT`
 */
EXPORTED int32_t PRE jsScanSystemPhaseInsertCameraConfiguration(
  jsScanSystem scan_system,
  jsScanHead scan_head,
  jsCamera camera,
  jsScanHeadConfiguration cfg) POST;

/**
 * @deprecated Use `jsScanSystemPhaseInsertConfigurationLaser`.
 *
 * @return `JS_ERROR_INVALID_ARGUMENT`
 */
EXPORTED int32_t PRE jsScanSystemPhaseInsertLaserConfiguration(
  jsScanSystem scan_system,
  jsScanHead scan_head,
  jsLaser laser,
  jsScanHeadConfiguration cfg) POST;

/**
 * @brief Obtains the minimum period that a given scan system can achieve
 * scanning when `jsScanSystemStartScanning` is called.
 *
 * @param scan_system Reference to system of scan heads.
 * @return The minimum scan period in microseconds on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int32_t PRE
jsScanSystemGetMinScanPeriod(
  jsScanSystem scan_system) POST;

/**
 * @brief Prepares the scan system to begin scanning. If connected, this
 * function will send all of the necessary configuration data to all of the
 * scan heads. Provided that no changes are made to any of the scan heads
 * associated with the scan system, the API will skip sending this data to the
 * scan heads when calling `jsScanSystemStartScanning` and allow scanning to
 * start faster.
 *
 * @param scan_system Reference to system of scan heads.
 * @return The minimum scan period in microseconds on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemConfigure(
  jsScanSystem scan_system) POST;

/**
 * @brief Obtains the configuration state of the scan system. If `false`, the
 * scan system needs to be configured by calling `jsScanSystemConfigure` or it
 * will be sent during `jsScanSystemStartScanning` call, incurring a time
 * penalty. If `true`, the system is configured and can start scanning with
 * no time penalty during call to `jsScanSystemStartScanning`.
 *
 * @param scan_system Reference to system of scan heads.
 * @return Boolean `true` if configured, `false` otherwise.
 */
EXPORTED bool PRE jsScanSystemIsConfigured(
  jsScanSystem scan_system) POST;

/**
 * @brief Commands scan heads in system to begin scanning, returning geometry
 * and/or brightness values to the client.
 *
 * @note The internal memory buffers of the scan heads will be cleared of all
 * old profile data upon start of scan. Ensure that all data from the previous
 * scan that is desired is read out before calling this function.
 *
 * @param scan_system Reference to system of scan heads.
 * @param period_us The scan period in microseconds.
 * @param fmt The data format of the returned scan profile data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemStartScanning(
  jsScanSystem scan_system,
  uint32_t period_us,
  jsDataFormat fmt) POST;

/**
 * @brief Commands scan heads in system to stop scanning.
 *
 * @param scan_system Reference to system of scan heads.
 * @return `0` on success, negative value `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemStopScanning(
  jsScanSystem scan_system) POST;

/**
 * @brief Commands scan heads in system to begin scanning, returning geometry
 * and/or brightness values to the client in an organized frame of profiles,
 * with each frame being comprised of one cycle through the phase table
 * corresponding with one complete period.
 *
 * @param scan_system Reference to system of scan heads.
 * @param period_us The scan period in microseconds.
 * @param fmt The data format of the returned scan profile data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemStartFrameScanning(
  jsScanSystem scan_system,
  uint32_t period_us,
  jsDataFormat fmt) POST;

/**
 * @brief Returns the number of profiles comprising a single frame of scan data.
 * This number should be used to appropriately size the arrays used to call
 * `jsScanSystemGetProfileFrame` and `jsScanSystemGetRawProfileFrame`.
 *
 * @param scan_system Reference to system of scan heads.
 * @return The number of elements in frame of scan data, or negative value
 * mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemGetProfilesPerFrame(
  jsScanSystem scan_system) POST;

/**
 * @brief Blocks until a frame of scan data is available to be read.
 *
 * @param scan_system Reference to system of scan heads.
 * @param timeout_us Maximum amount of time to wait for in microseconds.
 * @return `0` on timeout with no frames are available, positive value
 * indicating the total number of frames able to be read, or negative value
 * `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemWaitUntilFrameAvailable(
  jsScanSystem scan_system,
  uint32_t timeout_us) POST;

/**
 * @brief Checks if enough data has been collected to construct a frame of
 * profile data.
 *
 * @param scan_system Reference to system of scan heads.
 * @return Boolean `true` if frame is available, `false` otherwise.
 */
EXPORTED bool PRE jsScanSystemIsFrameAvailable(
  jsScanSystem scan_system) POST;

/**
 * @brief Empties the internal client side software buffers used to store
 * profiles for frame scanning.
 *
 * @note Under normal scanning conditions where the application consumes
 * frames as they become available, this function will not be needed. It's
 * use is to be found in cases where the application fails to consume frames
 * after some time and the number of buffered frames becomes more than the
 * application can consume and only the most recent scan frame is desired.
 *
 * @param scan_system Reference to system of scan heads.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemClearFrames(
  jsScanSystem scan_system) POST;

/**
 * @brief Reads one frame of `jsProfile` formatted profile data.
 *
 * @note If no frame is available this will return immediately.  Care should
 * be taken if this function is used in a loop; it is advised to either sleep
 * when `0` profiles are returned, or first call
 * `jsScanHeadWaitUntilFrameAvailable()` before `jsScanSystemGetProfileFrame()`
 * so as to avoid excessive CPU usage.
 *
 * @param scan_head Reference to scan head.
 * @param profiles Pointer to memory to store frame of profile data. Note, the
 * memory pointed to by `profiles` must be at least
 * `sizeof(jsProfile) * jsScanSystemGetProfilesPerFrame()` in total number of
 * bytes available.
 * @return The number of set profiles in frame on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemGetFrame(
  jsScanSystem scan_system,
  jsProfile *profiles) POST;

/**
 * @brief Reads one frame of `jsRawProfile` formatted profile data.
 *
 * @note If no frame is available this will return immediately.  Care should
 * be taken if this function is used in a loop; it is advised to either sleep
 * when `0` profiles are returned, or first call
 * `jsScanHeadWaitUntilFrameAvailable()` before `jsScanSystemGetProfileFrame()`
 * so as to avoid excessive CPU usage.
 *
 * @param scan_head Reference to scan head.
 * @param profiles Pointer to memory to store frame of profile data. Note, the
 * memory pointed to by `profiles` must be at least
 * `sizeof(jsRawProfile) * jsScanSystemGetProfilesPerFrame()` in total number
 * of bytes available.
 * @return The number of set profiles in frame on success, negative value
 * mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanSystemGetRawFrame(
  jsScanSystem scan_system,
  jsRawProfile *profiles) POST;

/**
 * @brief Gets scanning state for a scan system.
 *
 * @param scan_system Reference to system of scan heads.
 * @return Boolean `true` if scanning, `false` if not scanning.
 */
EXPORTED bool PRE jsScanSystemIsScanning(
  jsScanSystem scan_system) POST;

/**
 * @brief Obtains the product type of a given scan head.
 *
 * @note This function can only be called when a scan head is successfully
 * connected after calling `jsScanSystemConnect()`.
 *
 * @param scan_head Reference to scan head.
 * @return The enumerated scan head type.
 */
EXPORTED jsScanHeadType PRE jsScanHeadGetType(
  jsScanHead scan_head) POST;

/**
 * @brief Obtains the ID of the scan head.
 *
 * @param scan_head Reference to scan head.
 * @return The numeric ID of the scan head.
 */
EXPORTED uint32_t PRE jsScanHeadGetId(
  jsScanHead scan_head) POST;

/**
 * @brief Obtains the serial number of the scan head.
 *
 * @param scan_head Reference to scan head.
 * @return The serial number of the scan head.
 */
EXPORTED uint32_t PRE jsScanHeadGetSerial(
  jsScanHead scan_head) POST;

/**
 * @brief Obtains the capabilities for a given scan head.
 *
 * @param type The scan head product type to obtain capabilities for.
 * @param capabilities Pointer to struct to be filled with capabilities.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetCapabilities(
  jsScanHead scan_head,
  jsScanHeadCapabilities *capabilities) POST;

/**
 * @brief Obtains the firmware version of the scan head presented as unsigned
 * integer values.
 *
 * @param major The major version number.
 * @param minor The minor version number.
 * @param patch The patch version number.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetFirmwareVersion(
  jsScanHead scan_head,
  uint32_t *major,
  uint32_t *minor,
  uint32_t *patch) POST;

/**
 * @brief Obtains the connection state of a given scan head.
 *
 * @param scan_head Reference to scan head.
 * @return Boolean `true` on connected, `false` otherwise.
 */
EXPORTED bool PRE jsScanHeadIsConnected(
  jsScanHead scan_head) POST;

/**
 * @brief Configures the scan head according to the parameters specified.
 *
 * @note The configuration settings are sent to the scan head during the call
 * to `jsScanSystemStartScanning()`.
 *
 * @param scan_head Reference to scan head to be configured.
 * @param cfg The `jsScanHeadConfiguration` to be applied.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetConfiguration(
  jsScanHead scan_head,
  jsScanHeadConfiguration *cfg) POST;

/**
 * @brief Gets the scan head's configuration settings configured by the
 * `jsScanHeadConfiguration` function.
 *
 * @param scan_head Reference to scan head to be configured.
 * @param cfg The `jsScanHeadConfiguration` to be updated with current settings
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetConfiguration(
  jsScanHead scan_head,
  jsScanHeadConfiguration *cfg) POST;

/**
 * @brief Gets the safe default configuration for a given scan head.
 *
 * @param scan_head Reference to scan head to be configured.
 * @param cfg The `jsScanHeadConfiguration` to be updated with default.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetConfigurationDefault(
  jsScanHead scan_head,
  jsScanHeadConfiguration *cfg) POST;

/**
 * @brief Gets the safe default configuration for a given scan head.
 *
 * @param scan_head Reference to scan head to be configured.
 * @param cfg The `jsScanHeadConfiguration` to be updated with default.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetCableOrientation(
  jsScanHead scan_head,
  jsCableOrientation cable_orientation);

/**
 * @brief Gets the safe default configuration for a given scan head.
 *
 * @param scan_head Reference to scan head to be configured.
 * @param cfg The `jsScanHeadConfiguration` to be updated with default.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetCableOrientation(
  jsScanHead scan_head,
  jsCableOrientation *cable_orientation);

/**
 * @brief Configures spatial parameters of the scan head in order to properly
 * transform the data from a camera based coordinate system to one based on
 * mill placement.
 *
 * @note The alignment settings are sent to the scan head during the call to
 * `jsScanSystemConnect`.
 *
 * @param scan_head Reference to scan head.
 * @param roll_degrees The rotation in degrees to be applied to the Z axis.
 * @param shift_x The shift to be applied specified in scan system units.
 * @param shift_y The shift to be applied specified in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetAlignment(
  jsScanHead scan_head,
  double roll_degrees,
  double shift_x,
  double shift_y) POST;

/**
 * @brief Configures spatial parameters of the scan head in order to properly
 * transform the data from a camera based coordinate system to one based on
 * mill placement. This function is similar to `jsScanHeadSetAlignment`
 * except that it only applies to one camera rather than both.
 *
 * @note The alignment settings are sent to the scan head during the call to
 * `jsScanSystemConnect`.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera to apply parameters to.
 * @param roll_degrees The rotation in degrees to be applied.
 * @param shift_x The shift to be applied specified in scan system units.
 * @param shift_y The shift to be applied specified in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetAlignmentCamera(
  jsScanHead scan_head,
  jsCamera camera,
  double roll_degrees,
  double shift_x,
  double shift_y) POST;

/**
 * @brief Obtains the currently applied alignment settings.
 *
 * @note If configured using `jsScanHeadSetAlignment`, each camera will have
 * the same alignment settings.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera to get settings from.
 * @param roll_degrees Variable to hold roll in degrees.
 * @param shift_x Variable to hold shift in scan system units.
 * @param shift_y Variable to hold shift in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetAlignmentCamera(
  jsScanHead scan_head,
  jsCamera camera,
  double *roll_degrees,
  double *shift_x,
  double *shift_y) POST;

/**
 * @brief Configures spatial parameters of the scan head in order to properly
 * transform the data from a camera based coordinate system to one based on
 * mill placement. This function is similar to `jsScanHeadSetAlignment`
 * except that it only applies to one laser rather than all.
 *
 * @note The alignment settings are sent to the scan head during the call to
 * `jsScanSystemConnect`.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser to apply parameters to.
 * @param roll_degrees The rotation in degrees to be applied.
 * @param shift_x The shift to be applied specified in scan system units.
 * @param shift_y The shift to be applied specified in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetAlignmentLaser(
  jsScanHead scan_head,
  jsLaser laser,
  double roll_degrees,
  double shift_x,
  double shift_y) POST;

/**
 * @brief Obtains the currently applied alignment settings.
 *
 * @note If configured using `jsScanHeadSetAlignment`, each laser will have
 * the same alignment settings.
 *
 * @param scan_head Reference to scan head.
 * @param Laser The laser to get settings from.
 * @param roll_degrees Variable to hold roll in degrees.
 * @param shift_x Variable to hold shift in scan system units.
 * @param shift_y Variable to hold shift in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetAlignmentLaser(
  jsScanHead scan_head,
  jsLaser laser,
  double *roll_degrees,
  double *shift_x,
  double *shift_y) POST;

/**
 * @brief Allows specific pixels to be masked from scanning. This is useful to
 * help exclude regions in the field of view that generate spurious data due to
 * competing light sources. In order to exclude a given pixel, set the
 * appropriate entry in the `jsExclusionMask` struct to a non-zero value.
 *
 * @note Only supported by JS-50 scan heads running v16.1.0 firmware or greater.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera to apply exclusion mask to.
 * @param mask Pointer to mask of pixels to exclude.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetExclusionMaskCamera(
  jsScanHead scan_head,
  jsCamera camera,
  jsExclusionMask *mask) POST;

/**
 * @brief Allows specific pixels to be masked from scanning. This is useful to
 * help exclude regions in the field of view that generate spurious data due to
 * competing light sources. In order to exclude a given pixel, set the
 * appropriate entry in the `jsExclusionMask` struct to a non-zero value.
 *
 * @note Only supported by JS-50 scan heads running v16.1.0 firmware or greater.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser to apply exclusion mask to.
 * @param mask Pointer to mask of pixels to exclude.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetExclusionMaskLaser(
  jsScanHead scan_head,
  jsLaser laser,
  jsExclusionMask *mask) POST;

/**
 * @brief Gets the current exclusion mask applied.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera the exclusion mask is applied to.
 * @param mask Pointer to be filled with current exclusion mask.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetExclusionMaskCamera(
  jsScanHead scan_head,
  jsCamera camera,
  jsExclusionMask *mask) POST;

/**
 * @brief Gets the current exclusion mask applied.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser the exclusion mask is applied to.
 * @param mask Pointer to be filled with current exclusion mask.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetExclusionMaskLaser(
  jsScanHead scan_head,
  jsLaser laser,
  jsExclusionMask *mask) POST;

/**
 * @brief Allows adjusting the scan data's brightness values. This can be
 * useful to help ensure uniformity across all columns of the scan data.
 *
 * @note This is a beta feature that may be changed in the future. It is
 * is offered here to provide access to functionality that may prove useful to
 * end users and allow them to submit feedback back to JoeScan. In a future
 * release, this code may change; care should be taken when adding to
 * applications. For any questions, reach out to a JoeScan representative for
 * guidance.
 *
 * @note Only supported by JS-50 scan heads running v16.1.0 firmware or greater.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera to apply exclusion mask to.
 * @param correction Pointer to brightness correction to apply.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetBrightnessCorrectionCamera_BETA(
  jsScanHead scan_head,
  jsCamera camera,
  jsBrightnessCorrection_BETA *correction) POST;

/**
 * @brief Allows adjusting the scan data's brightness values. This can be
 * useful to help ensure uniformity across all columns of the scan data.
 *
 * @note This is a beta feature that may be changed in the future. It is
 * is offered here to provide access to functionality that may prove useful to
 * end users and allow them to submit feedback back to JoeScan. In a future
 * release, this code may change; care should be taken when adding to
 * applications. For any questions, reach out to a JoeScan representative for
 * guidance.
 *
 * @note Only supported by JS-50 scan heads running v16.1.0 firmware or greater.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser to apply exclusion mask to.
 * @param correction Pointer to brightness correction to apply.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetBrightnessCorrectionLaser_BETA(
  jsScanHead scan_head,
  jsLaser laser,
  jsBrightnessCorrection_BETA *correction) POST;

/**
 * @brief Gets the current brightness correction values applied.
 *
 * @note This is a beta feature that may be changed in the future. It is
 * is offered here to provide access to functionality that may prove useful to
 * end users and allow them to submit feedback back to JoeScan. In a future
 * release, this code may change; care should be taken when adding to
 * applications. For any questions, reach out to a JoeScan representative for
 * guidance.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera the exclusion mask is applied to.
 * @param correction Pointer to be filled with current brightness correction.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetBrightnessCorrectionCamera_BETA(
  jsScanHead scan_head,
  jsCamera camera,
  jsBrightnessCorrection_BETA *correction) POST;

/**
 * @brief Gets the current brightness correction values applied.
 *
 * @note This is a beta feature that may be changed in the future. It is
 * is offered here to provide access to functionality that may prove useful to
 * end users and allow them to submit feedback back to JoeScan. In a future
 * release, this code may change; care should be taken when adding to
 * applications. For any questions, reach out to a JoeScan representative for
 * guidance.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser the exclusion mask is applied to.
 * @param correction Pointer to be filled with current brightness correction.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetBrightnessCorrectionLaser_BETA(
  jsScanHead scan_head,
  jsLaser laser,
  jsBrightnessCorrection_BETA *correction) POST;

/**
 * @brief Configures the scan head to only return a profile after the specified
 * number of ticks have occurred on `JS_ENCODER_MAIN`.
 *
 * @note Setting this value to zero, the default, will result in profiles
 * always being returned irregardless of the encoder travel.
 *
 * @note Use `jsScanHeadSetIdleScanPeriod` to configure the scan head to return
 * profiles at a reduced rate when the encoder has not traveled enough to
 * trigger a new profile to be returned.
 *
 * @note This feature is currently not supported with frame scanning.
 *
 * @param scan_head Reference to scan head.
 * @param min_encoder_travel Number of encoder ticks needed for new profile.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetMinimumEncoderTravel(
  jsScanHead scan_head,
  uint32_t min_encoder_travel);

/**
 * @brief Returns the configured encoder travel value set by
 * `jsScanHeadSetMinimumEncoderTravel`.
 *
 * @note This feature is currently not supported with frame scanning.
 *
 * @param scan_head Reference to scan head.
 * @param min_encoder_travel Pointer to store configured travel value.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetMinimumEncoderTravel(
  jsScanHead scan_head,
  uint32_t *min_encoder_travel);

/**
 * @brief Gonfigures the duration by which new profiles are returned to the
 * user when the encoder travel value specified by
 * `jsScanHeadSetMinimumEncoderTravel` has not been met.
 *
 * @note Setting this value to zero, the default will result in no profiles
 * being returned until the encoder has traveled the specified distance.
 *
 * @note The resolution of the idle scan period is currently only in increments
 * of the scan system's scan period and the number of times the scan head is
 * scheduled in the phase table. Greater granularity will be achievable in a
 * future release.
 *
 * @note This feature is currently not supported with frame scanning.
 *
 * @param scan_head Reference to scan head.
 * @param idle_period_us The idle scan period in microseconds
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetIdleScanPeriod(
  jsScanHead scan_head,
  uint32_t idle_period_us);

/**
 * @brief Returns the configured duration set by `jsScanHeadSetIdleScanPeriod`.
 *
 * @note This feature is currently not supported with frame scanning.
 *
 * @param scan_head Reference to scan head.
 * @param idle_period_us Pointer to store configured idle period value.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetIdleScanPeriod(
  jsScanHead scan_head,
  uint32_t *idle_period_us);

/**
 * @brief Sets a rectangular scan window for a scan head to restrict its
 * field of view when scanning.
 *
 * @param scan_head Reference to scan head.
 * @param window_top The top window dimension in scan system units.
 * @param window_bottom The bottom window dimension in scan system units.
 * @param window_left The left window dimension in scan system units.
 * @param window_right The right window dimension in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetWindowRectangular(
  jsScanHead scan_head,
  double window_top,
  double window_bottom,
  double window_left,
  double window_right) POST;

/**
 * @brief Sets a rectangular scan window for a particular scan head's camera to
 * restrict its field of view when scanning.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera to apply the window to.
 * @param window_top The top window dimension in scan system units.
 * @param window_bottom The bottom window dimension in scan system units.
 * @param window_left The left window dimension in scan system units.
 * @param window_right The right window dimension in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetWindowRectangularCamera(
  jsScanHead scan_head,
  jsCamera camera,
  double window_top,
  double window_bottom,
  double window_left,
  double window_right) POST;

/**
 * @brief Sets a rectangular scan window for a particular scan head's laser to
 * restrict its field of view when scanning.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser to apply the window to.
 * @param window_top The top window dimension in scan system units.
 * @param window_bottom The bottom window dimension in scan system units.
 * @param window_left The left window dimension in scan system units.
 * @param window_right The right window dimension in scan system units.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetWindowRectangularLaser(
  jsScanHead scan_head,
  jsLaser laser,
  double window_top,
  double window_bottom,
  double window_left,
  double window_right) POST;

/**
 * @brief Sets a user defined polygonal scan window for a scan head to restrict
 * its field of view when scanning. The points must be ordered in a clockwise
 * fashion and the resulting shape must be convex. The first and last points
 * will be automatically connected, removing the need to duplicate the first
 * point at the end of 'points'.
 *
 * @param scan_head Reference to scan head.
 * @param points Array of coordinates defining the X/Y points of the polygon.
 * @param points_len The number of points in the `points` array.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetPolygonWindow(
  jsScanHead scan_head,
  jsCoordinate *points,
  uint32_t points_len);

/**
 * @brief Sets a user defined polygonal scan window for a particular scan
 * head's camera to restrict its field of view when scanning. The points must
 * be ordered in a clockwise fashion and the resulting shape must be convex.
 * The first and last points will be automatically connected, removing the need
 * to duplicate the first point at the end of 'points'.
 *
 * @param scan_head Reference to scan head.
 * @param camera The camera to apply the window to.
 * @param points Array of coordinates defining the X/Y points of the polygon.
 * @param points_len The number of points in the `points` array.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetPolygonWindowCamera(
  jsScanHead scan_head,
  jsCamera camera,
  jsCoordinate *points,
  uint32_t points_len);

/**
 * @brief Sets a user defined polygonal scan window for a particular scan
 * head's laser to restrict its field of view when scanning. The points must
 * be ordered in a clockwise fashion and the resulting shape must be convex.
 * The first and last points will be automatically connected, removing the need
 * to duplicate the first point at the end of 'points'.
 *
 * @param scan_head Reference to scan head.
 * @param laser The laser to apply the window to.
 * @param points Array of coordinates defining the X/Y points of the polygon.
 * @param points_len The number of points in the `points` array.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadSetPolygonWindowLaser(
  jsScanHead scan_head,
  jsLaser laser,
  jsCoordinate *points,
  uint32_t points_len);

/**
 * @brief Reads the last reported status update from a scan head.
 *
 * @param scan_head Reference to scan head.
 * @param status Pointer to be updated with status contents.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetStatus(
  jsScanHead scan_head,
  jsScanHeadStatus *status) POST;

/**
 * @brief Obtains the number of profiles currently available to be read out from
 * a given scan head.
 *
 * @param scan_head Reference to scan head.
 * @return The total number of profiles able to be read on success, negative
 * value `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetProfilesAvailable(
  jsScanHead scan_head) POST;

/**
 * @brief Blocks until the number of requested profiles are avilable to be read
 * out from a given scan head.
 *
 * @param scan_head Reference to scan head.
 * @param count The number of profiles to wait for. Should not exceed
 * `JS_SCAN_HEAD_PROFILES_MAX`.
 * @param timeout_us Maximum amount of time to wait for in microseconds.
 * @return `0` on timeout with no profiles available, positive value indicating
 * the total number of profiles able to be read after `count` or `timeout_us` is
 * reached, or negative value `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadWaitUntilProfilesAvailable(
  jsScanHead scan_head,
  uint32_t count,
  uint32_t timeout_us) POST;

/**
 * @brief Empties the internal client side software buffers used to store
 * profiles received from a given scan head.
 *
 * @note Under normal scanning conditions where the application consumes
 * profiles as they become available, this function will not be needed. It's
 * use is to be found in cases where the application fails to consume profiles
 * after some time and the number of buffered profiles, as indicated by the
 * `jsScanHeadGetProfilesAvailable` function becomes more than the application
 * can consume and only the most recent scan data is desired.
 *
 * @param scan_head Reference to scan head.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadClearProfiles(
  jsScanHead scan_head) POST;

/**
 * @brief Reads `jsProfile` formatted profile data from a given scan head.
 * The number of profiles returned is either the max value requested or the
 * total number of profiles ready to be read out, whichever is less.
 *
 * @note If no profiles are available this will return immediately.  Care should
 * be taken if this function is used in a loop; it is advised to either sleep
 * when `0` profiles are returned, or first call
 * `jsScanHeadWaitUntilProfilesAvailable()` before `jScanHeadGetProfiles()` so
 * as to avoid excessive CPU usage.
 *
 * @param scan_head Reference to scan head.
 * @param profiles Pointer to memory to store profile data. Note, the memory
 * pointed to by `profiles` must be at least `sizeof(jsProfile) * max_profiles`
 * in total number of bytes available.
 * @param max_profiles The maximum number of profiles to read. Should not
 * exceed `JS_SCAN_HEAD_PROFILES_MAX`.
 * @return The number of profiles read on success, negative value mapping to
 * `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetProfiles(
  jsScanHead scan_head,
  jsProfile *profiles,
  uint32_t max_profiles) POST;

/**
 * @brief Reads `jsRawProfile` formatted profile data from a given scan head.
 * The number of profiles returned is either the max value requested or the
 * total number of profiles ready to be read out, whichever is less.
 *
 * @param scan_head Reference to scan head.
 * @param profiles Pointer to memory to store profile data. Note, the memory
 * pointed to by `profiles` must be at least `sizeof(jsProfile) * max_profiles`
 * in total number of bytes available.
 * @param max_profiles The maximum number of profiles to read. Should not
 * exceed `JS_SCAN_HEAD_PROFILES_MAX`.
 * @return The number of profiles read on success, negative value mapping to
 * `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetRawProfiles(
  jsScanHead scan_head,
  jsRawProfile *profiles,
  uint32_t max_profiles) POST;

/**
 * @brief Obtains a single camera profile from a scan head to be used for
 * diagnostic purposes.
 *
 * @note This function should be called after `jsScanSystemConnect()`, but
 * not while the system is set to scan by calling
 * `jsScanSystemStartScanning()`.
 *
 * @note This function will automatically select the correct camera / laser
 * pair, as used when scanning, when performing the capture.
 *
 * @note Only `JS_DIAGNOSTIC_FIXED_EXPOSURE` is supported; in a future release,
 * `JS_DIAGNOSTIC_AUTO_EXPOSURE` will be supported.
 *
 * @param scan_head Reference to scan head.
 * @param camera Camera to use for profile capture.
 * @param mode `JS_DIAGNOSTIC_FIXED_EXPOSURE` to use the laser on time and
 * camera exposure provided as function arguments, `JS_DIAGNOSTIC_AUTO_EXPOSURE`
 * to dynamically adjust camera & laser according to `jsScanHeadConfiguration`
 * provided to the `jsScanHeadSetConfiguration` function.
 * @param laser_on_time_us Time laser is on in microseconds.
 * @param camera_exposure_time_us Time camera exposes in microseconds.
 * @param profile Pointer to memory to store profile data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetDiagnosticProfileCamera(
  jsScanHead scan_head,
  jsCamera camera,
  jsDiagnosticMode mode,
  uint32_t laser_on_time_us,
  uint32_t camera_exposure_time_us,
  jsRawProfile *profile) POST;

/**
 * @brief Obtains a single camera profile from a scan head to be used for
 * diagnostic purposes.
 *
 * @note This function should be called after `jsScanSystemConnect()`, but
 * not while the system is set to scan by calling
 * `jsScanSystemStartScanning()`.
 *
 * @note This function will automatically select the correct camera / laser
 * pair, as used when scanning, when performing the capture.
 *
 * @note Only `JS_DIAGNOSTIC_FIXED_EXPOSURE` is supported; in a future release,
 * `JS_DIAGNOSTIC_AUTO_EXPOSURE` will be supported.
 *
 * @param scan_head Reference to scan head.
 * @param laser Laser to use for profile capture.
 * @param mode `JS_DIAGNOSTIC_FIXED_EXPOSURE` to use the laser on time and
 * camera exposure provided as function arguments, `JS_DIAGNOSTIC_AUTO_EXPOSURE`
 * to dynamically adjust camera & laser according to `jsScanHeadConfiguration`
 * provided to the `jsScanHeadSetConfiguration` function.
 * @param laser_on_time_us Time laser is on in microseconds.
 * @param camera_exposure_time_us Time camera exposes in microseconds.
 * @param profile Pointer to memory to store profile data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetDiagnosticProfileLaser(
  jsScanHead scan_head,
  jsLaser laser,
  jsDiagnosticMode mode,
  uint32_t laser_on_time_us,
  uint32_t camera_exposure_time_us,
  jsRawProfile *profile) POST;

/**
 * @brief Obtains a single camera image from a scan head to be used for
 * diagnostic purposes.
 *
 * @note This function should be called after `jsScanSystemConnect()`, but
 * not after the system has been set to scan by calling
 * `jsScanSystemStartScanning()`.
 *
 * @note This function will automatically select the correct camera / laser
 * pair, as used when scanning, when performing the capture.
 *
 * @deprecated Will be removed in a future release.
 *
 * @param scan_head Reference to scan head.
 * @param camera Camera to use for image capture. The laser to be in view of
 * the image will be chosen based on the chosen camera.
 * @param mode Must be set to `JS_DIAGNOSTIC_FIXED_EXPOSURE`.
 * @param laser_on_time_us Time laser is on in microseconds.
 * @param camera_exposure_time_us Time camera exposes in microseconds.
 * @param image Pointer to memory to store camera image data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetDiagnosticImageCamera(
  jsScanHead scan_head,
  jsCamera camera,
  jsDiagnosticMode mode,
  uint32_t laser_on_time_us,
  uint32_t camera_exposure_time_us,
  jsCameraImage *image) POST;

/**
 * @brief Obtains a single camera image from a scan head to be used for
 * diagnostic purposes.
 *
 * @note This function should be called after `jsScanSystemConnect()`, but
 * not after the system has been set to scan by calling
 * `jsScanSystemStartScanning()`.
 *
 * @note This function will automatically select the correct camera / laser
 * pair, as used when scanning, when performing the capture.
 *
 * @deprecated Will be removed in a future release.
 *
 * @param scan_head Reference to scan head.
 * @param laser Laser to be in view of image capture. The camera that takes the
 * image will automatically be chosen based on the chosen laser.
 * @param mode Must be set to `JS_DIAGNOSTIC_FIXED_EXPOSURE`.
 * @param laser_on_time_us Time laser is on in microseconds.
 * @param camera_exposure_time_us Time camera exposes in microseconds.
 * @param image Pointer to memory to store camera image data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetDiagnosticImageLaser(
  jsScanHead scan_head,
  jsLaser laser,
  jsDiagnosticMode mode,
  uint32_t laser_on_time_us,
  uint32_t camera_exposure_time_us,
  jsCameraImage *image) POST;

/**
 * @brief Obtains a single camera image from a scan head to be used for
 * diagnostic purposes.
 *
 * @note This function should be called after `jsScanSystemConnect()`, but
 * not after the system has been set to scan by calling
 * `jsScanSystemStartScanning()`.
 *
 * @deprecated Will be removed in a future release.
 *
 * @param scan_head Reference to scan head.
 * @param camera Camera to use for image capture.
 * @param laser Laser to be in view of image capture.
 * @param mode Must be set to `JS_DIAGNOSTIC_FIXED_EXPOSURE`.
 * @param laser_on_time_us Time laser is on in microseconds.
 * @param camera_exposure_time_us Time camera exposes in microseconds.
 * @param image Pointer to memory to store camera image data.
 * @return `0` on success, negative value mapping to `jsError` on error.
 */
EXPORTED int32_t PRE jsScanHeadGetDiagnosticImage(
  jsScanHead scan_head,
  jsCamera camera,
  jsLaser laser,
  jsDiagnosticMode mode,
  uint32_t laser_on_time_us,
  uint32_t camera_exposure_time_us,
  jsCameraImage *image) POST;

#ifdef PRE
  #undef PRE
#endif

#ifdef POST
  #undef POST
#endif

#endif

#ifdef __cplusplus
} // extern "C" {
#endif

#endif
