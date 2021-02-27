/*!
 * \file SickLMS2xx.hh
 * \brief Definition of class SickLMS2xx.
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_LMS_2XX_HH
#define SICK_LMS_2XX_HH

/* Implementation dependencies */
#include <string>
#include <iostream>
#include <termios.h>

#include "SickLIDAR.hh"
#include "SickException.hh"

#include "SickLMS2xxBufferMonitor.hh"
#include "SickLMS2xxMessage.hh"

/* Macro definitions */
#define DEFAULT_SICK_LMS_2XX_SICK_BAUD                                       (B9600)  ///< Initial baud rate of the LMS (whatever is set in flash)
#define DEFAULT_SICK_LMS_2XX_HOST_ADDRESS                                     (0x80)  ///< Client/host default serial address
#define DEFAULT_SICK_LMS_2XX_SICK_ADDRESS                                     (0x00)  ///< Sick LMS default serial address
#define DEFAULT_SICK_LMS_2XX_SICK_PASSWORD                                "SICK_LMS"  ///< Password for entering installation mode
#define DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT                (unsigned int)(1e6)  ///< The max time to wait for a message reply (usecs)
#define DEFAULT_SICK_LMS_2XX_SICK_SWITCH_MODE_TIMEOUT            (unsigned int)(3e6)  ///< Can take the Sick LD up to 3 seconds to reply (usecs)
#define DEFAULT_SICK_LMS_2XX_SICK_MEAN_VALUES_MESSAGE_TIMEOUT   (unsigned int)(15e6)  ///< When using mean values, the Sick can sometimes take more than 10s to respond
#define DEFAULT_SICK_LMS_2XX_SICK_CONFIG_MESSAGE_TIMEOUT        (unsigned int)(15e6)  ///< The sick can take some time to respond to config commands (usecs)
#define DEFAULT_SICK_LMS_2XX_BYTE_INTERVAL                                      (55)  ///< Minimum time in microseconds between transmitted bytes
#define DEFAULT_SICK_LMS_2XX_NUM_TRIES                                           (3)  ///< The max number of tries before giving up on a request
    
/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A general class for interfacing w/ SickLMS2xx2xx laser range finders
   *
   * This class implements the basic telegram protocol for SickLMS2xx2xx range finders.
   * It allows the setting of such parameters as angular resolution, fov, etc...
   */
  class SickLMS2xx : public SickLIDAR< SickLMS2xxBufferMonitor, SickLMS2xxMessage >
  { 

  public:
    
    /** Define the maximum number of measurements */
    static const uint16_t SICK_MAX_NUM_MEASUREMENTS = 721;                     ///< Maximum number of measurements returned by the Sick LMS

    /*!
     * \enum sick_lms_2xx_type_t 
     * \brief Defines the Sick LMS 2xx types.
     * This enum lists all of the supported Sick LMS models.
     */
    enum sick_lms_2xx_type_t {
      
      /* Supported 200 models */
      SICK_LMS_TYPE_200_30106,                                                 ///< Sick LMS type 200-30106
      
      /* Supported 211 models */
      SICK_LMS_TYPE_211_30106,                                                 ///< Sick LMS type 211-30106
      SICK_LMS_TYPE_211_30206,                                                 ///< Sick LMS type 211-30206
      SICK_LMS_TYPE_211_S07,                                                   ///< Sick LMS type 211-S07
      SICK_LMS_TYPE_211_S14,                                                   ///< Sick LMS type 211-S14
      SICK_LMS_TYPE_211_S15,                                                   ///< Sick LMS type 211-S15
      SICK_LMS_TYPE_211_S19,                                                   ///< Sick LMS type 211-S19
      SICK_LMS_TYPE_211_S20,                                                   ///< Sick LMS type 211-S20

      /* Supported 220 models */
      SICK_LMS_TYPE_220_30106,                                                 ///< Sick LMS type 220-30106

      /* Supported 221 models */
      SICK_LMS_TYPE_221_30106,                                                 ///< Sick LMS type 221-30106
      SICK_LMS_TYPE_221_30206,                                                 ///< Sick LMS type 221-30206
      SICK_LMS_TYPE_221_S07,                                                   ///< Sick LMS type 221-S07
      SICK_LMS_TYPE_221_S14,                                                   ///< Sick LMS type 221-S14
      SICK_LMS_TYPE_221_S15,                                                   ///< Sick LMS type 221-S15
      SICK_LMS_TYPE_221_S16,                                                   ///< Sick LMS type 221-S16
      SICK_LMS_TYPE_221_S19,                                                   ///< Sick LMS type 221-S19
      SICK_LMS_TYPE_221_S20,                                                   ///< Sick LMS type 221-S20

      /* Supported 291 models */
      SICK_LMS_TYPE_291_S05,                                                   ///< Sick LMS type 291-S05
      SICK_LMS_TYPE_291_S14,                                                   ///< Sick LMS type 291-S14 (LMS Fast)
      SICK_LMS_TYPE_291_S15,                                                   ///< Sick LMS type 291-S15

      /* Any unknown model */
      SICK_LMS_TYPE_UNKNOWN = 0xFF                                             ///< Unknown sick type      

    };
    
    /*!
     * \enum sick_lms_2xx_variant_t
     * \brief Defines the Sick LMS 2xx variant type.
     */
    enum sick_lms_2xx_variant_t {
      SICK_LMS_VARIANT_2XX_TYPE_6 = 0x00,                                      ///< Standard LMS 2xx type 6 models
      SICK_LMS_VARIANT_SPECIAL = 0x01,                                         ///< Special models (i.e. LMS211-/221-S19/-S20
      SICK_LMS_VARIANT_UNKNOWN = 0xFF                                          ///< Unknown LMS variant
    };

    /*!
     * \enum sick_lms_2xx_scan_angle_t
     * \brief Defines the scan angle for the Sick LMS 2xx.
     */
    enum sick_lms_2xx_scan_angle_t {
      SICK_SCAN_ANGLE_90 = 90,                                                 ///< Scanning angle of 90 degrees
      SICK_SCAN_ANGLE_100 = 100,                                               ///< Scanning angle of 100 degrees
      SICK_SCAN_ANGLE_180 = 180,                                               ///< Scanning angle of 180 degrees
      SICK_SCAN_ANGLE_UNKNOWN = 0xFF                                           ///< Unknown scanning angle
    };
    
    /*!
     * \enum sick_lms_2xx_scan_resolution_t
     * \brief Defines the available resolution settings for the Sick LMS 2xx.
     */
    enum sick_lms_2xx_scan_resolution_t {
      SICK_SCAN_RESOLUTION_25 = 25,                                            ///< 0.25 degree angular resolution
      SICK_SCAN_RESOLUTION_50 = 50,                                            ///< 0.50 degree angular resolution
      SICK_SCAN_RESOLUTION_100 = 100,                                          ///< 1.00 degree angular resolution
      SICK_SCAN_RESOLUTION_UNKNOWN = 0xFF                                      ///< Unknown angular resolution
    };

    /*!
     * \enum sick_lms_2xx_measuring_units_t
     * \brief Defines the available Sick LMS 2xx measured value units.
     */
    enum sick_lms_2xx_measuring_units_t {
      SICK_MEASURING_UNITS_CM = 0x00,                                          ///< Measured values are in centimeters
      SICK_MEASURING_UNITS_MM = 0x01,                                          ///< Measured values are in milimeters
      SICK_MEASURING_UNITS_UNKNOWN = 0xFF                                      ///< Unknown units
    };

    /*!
     * \enum sick_lms_2xx_sensitivity_t
     * \brief Sick sensitivities. Only valid for Sick LMS 211/221/291!
     */
    enum sick_lms_2xx_sensitivity_t {
      SICK_SENSITIVITY_STANDARD = 0x00,                                        ///< Standard sensitivity: 30m @ 10% reflectivity
      SICK_SENSITIVITY_MEDIUM = 0x01,                                          ///< Medium sensitivity:   25m @ 10% reflectivity
      SICK_SENSITIVITY_LOW = 0x02,                                             ///< Low sensitivity:      20m @ 10% reflectivity
      SICK_SENSITIVITY_HIGH = 0x03,                                            ///< High sensitivity:     42m @ 10% reflectivity
      SICK_SENSITIVITY_UNKNOWN = 0xFF                                          ///< Sensitivity unknown
    };

    /*!
     * \enum sick_lms_2xx_peak_threshold_t
     * \brief Sick peak threshold. Only valid for Sick LMS 200/220!
     */
    enum sick_lms_2xx_peak_threshold_t {
      SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION = 0x00,            ///< Standard: peak threshold detection, no black extension
      SICK_PEAK_THRESHOLD_DETECTION_WITH_BLACK_EXTENSION = 0x01,               ///< Peak threshold detection, active black extension
      SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_NO_BLACK_EXTENSION = 0x02,         ///< No peak threshold detection, no black extension
      SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_BLACK_EXTENSION = 0x03,            ///< No peak threshold detection, active black extension
      SICK_PEAK_THRESHOLD_UNKNOWN = 0xFF                                       ///< Peak threshold unknown
    };

    /*!
     * \enum sick_lms_2xx_status_t
     * \brief Defines the status of the Sick LMS 2xx unit.
     */
    enum sick_lms_2xx_status_t {
      SICK_STATUS_OK = 0x00,                                                   ///< LMS is OK
      SICK_STATUS_ERROR = 0x01,                                                ///< LMS has encountered an error
      SICK_STATUS_UNKNOWN = 0xFF                                               ///< Unknown LMS status
    };
    
    /*!
     * \enum sick_lms_2xx_measuring_mode_t
     * \brief Defines the measurment modes supported by Sick LMS 2xx.
     */
    enum sick_lms_2xx_measuring_mode_t { 
      SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE = 0x00,                                ///< Measurement range 8m/80m; fields A,B and Dazzle (Default)
      SICK_MS_MODE_8_OR_80_REFLECTOR = 0x01,                                   ///< Measurement range 8/80m; reflector bits in 8 levels
      SICK_MS_MODE_8_OR_80_FA_FB_FC = 0x02,                                    ///< Measurement range 8/80m; fields A,B, and C
      SICK_MS_MODE_16_REFLECTOR = 0x03,                                        ///< Measurement range 16m; reflector bits in 4 levels
      SICK_MS_MODE_16_FA_FB = 0x04,                                            ///< Measurement range 16m; fields A and B
      SICK_MS_MODE_32_REFLECTOR = 0x05,                                        ///< Measurement range 32m; reflector bit in 2 levels
      SICK_MS_MODE_32_FA = 0x06,                                               ///< Measurement range 32m; field A
      SICK_MS_MODE_32_IMMEDIATE = 0x0F,                                        ///< Measurement range 32m; immediate data transmission, no flags      
      SICK_MS_MODE_REFLECTIVITY = 0x3F,                                        ///< Sick LMS 2xx returns reflectivity (echo amplitude) values instead of range measurements
      SICK_MS_MODE_UNKNOWN = 0xFF                                              ///< Unknown range
    };
    
    /*!
     * \enum sick_lms_2xx_operating_mode_t
     * \brief Defines the operating modes supported by Sick LMS 2xx. 
     * See page 41 of the LMS 2xx telegram manual for additional descriptions of these modes.
     */
    enum sick_lms_2xx_operating_mode_t {
      SICK_OP_MODE_INSTALLATION = 0x00,                                        ///< Installation mode for writing EEPROM
      SICK_OP_MODE_DIAGNOSTIC = 0x10,                                          ///< Diagnostic mode for testing purposes
      SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT = 0x20,           ///< Streams minimum measured values for each segement
      SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT = 0x21,                 ///< Sends the min measured values when object is detected
      SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT = 0x22,              ///< Streams min "vertical distance" to objects
      SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT = 0x23,             ///< Sends min vertical distance to object when detected
      SICK_OP_MODE_MONITOR_STREAM_VALUES = 0x24,                               ///< Streams all measured values in a scan 
      SICK_OP_MODE_MONITOR_REQUEST_VALUES = 0x25,                              ///< Sends measured range values on request (i.e. when polled)
      SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES = 0x26,                          ///< Streams mean values from a sample size of n consecutive scans
      SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE = 0x27,                      ///< Streams data from given subrange
      SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE = 0x28,                 ///< Streams mean values over requested subrange
      SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS = 0x29,                   ///< Streams measured values with associated flags
      SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN = 0x2A,             ///< Streams measured values of partial scan directly after measurement
      SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN = 0x2B,  ///< Streams range and intensity from n partial scans
      SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE = 0x2C, ///< Streams minimum measured values for each segment in a sub-range
      SICK_OP_MODE_MONITOR_NAVIGATION = 0x2E,                                  ///< Sick outputs navigation data records
      SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT = 0x50,                    ///< Streams measured range from a scan and sub-range of reflectivity values
      SICK_OP_MODE_UNKNOWN = 0xFF                                              ///< Unknown operating mode
    };

    /*!
     * \enum sick_lms_2xx_baud_t
     * \brief Defines available Sick LMS 2xx baud rates
     */
    enum sick_lms_2xx_baud_t {
      SICK_BAUD_9600 = 0x42,                                                   ///< 9600 baud
      SICK_BAUD_19200 = 0x41,                                                  ///< 19200 baud
      SICK_BAUD_38400 = 0x40,                                                  ///< 38400 baud
      SICK_BAUD_500K = 0x48,                                                   ///< 500000 baud
      SICK_BAUD_UNKNOWN = 0xFF                                                 ///< Unknown baud rate
    };

    /** Define Sick LMS 2xx availability levels */
    static const uint8_t SICK_FLAG_AVAILABILITY_DEFAULT = 0x00;                ///< Availability unspecified
    static const uint8_t SICK_FLAG_AVAILABILITY_HIGH = 0x01;                   ///< Highest availability (comparable to LMS types 1 to 5)
    static const uint8_t SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES = 0x02;      ///< Send real-time indices
    static const uint8_t SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT = 0x04;       ///< Dazzle evalutation has no effect on switching outputs
  
    /*!
     * \struct sick_lms_2xx_operating_status_tag
     * \brief A structure for aggregating the data that
     *        collectively defines the operating status
     *        of the device.
     */
    /*!
     * \typedef sick_lms_2xx_operating_status_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_operating_status_tag {
      uint16_t sick_scan_angle;                                                ///< Sick scanning angle (deg)
      uint16_t sick_scan_resolution;                                           ///< Sick angular resolution (1/100 deg)
      uint16_t sick_num_motor_revs;                                            ///< Sick number of motor revs
      uint8_t sick_operating_mode;                                             ///< Sick operating mode
      uint8_t sick_measuring_mode;                                             ///< Sick measuring mode
      uint8_t sick_laser_mode;                                                 ///< Sick laser is on/off
      uint8_t sick_device_status;                                              ///< Sick device status {ok,error}
      uint8_t sick_measuring_units;                                            ///< Sick measuring units {cm,mm}
      uint8_t sick_address;                                                    ///< Sick device address
      uint8_t sick_variant;                                                    ///< Sick variant {special,standard}
    } sick_lms_2xx_operating_status_t;
    
    /*!
     * \struct sick_lms_2xx_software_status_tag
     * \brief A structure for aggregating the data that
     *        collectively defines the system software
     *        for the Sick LMS 2xx unit.
     */
    /*!
     * \typedef sick_lms_2xx_software_status_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_software_status_tag {
      uint8_t sick_system_software_version[8];                                 ///< Sick system software version
      uint8_t sick_prom_software_version[8];                                   ///< Sick boot prom software version
    } sick_lms_2xx_software_status_t;
    
    /*!
     * \struct sick_lms_2xx_restart_status_tag
     * \brief A structure for aggregating the data that
     *        collectively defines the system restart
     *        config for the Sick LMS 2xx unit
     */
    typedef struct sick_lms_2xx_restart_status_tag {
      uint16_t sick_restart_time;                                              ///< Sick restart time  
      uint8_t sick_restart_mode;                                               ///< Sick restart mode
    } sick_lms_2xx_restart_status_t;
    
    /*!
     * \struct sick_lms_2xx_pollution_status_tag
     * \brief A structure for aggregating the data that
     *        collectively defines the pollution values
     *        and settings for the device
     */
    /*!
     * \typedef sick_lms_2xx_pollution_status_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_pollution_status_tag {
      uint16_t sick_pollution_vals[8];                                         ///< Calibrating the pollution channels
      uint16_t sick_pollution_calibration_vals[8];                             ///< Calibrating the pollution channel values
      uint16_t sick_reference_pollution_vals[4];                               ///< Reference pollution values
      uint16_t sick_reference_pollution_calibration_vals[4];                   ///< Reference pollution calibration values
    } sick_lms_2xx_pollution_status_t;
    
    /*!
     * \struct sick_lms_2xx_signal_status_tag
     * \brief A structure for aggregating the data that
     *        collectively define the signal config and
     *        status.
     */
    /*!
     * \typedef sick_lms_2xx_signal_status_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_signal_status_tag {
      uint16_t sick_reference_scale_1_dark_100;                                ///< Receive signal amplitude in ADC incs when reference signal is switched off (Signal 1, Dark 100%)
      uint16_t sick_reference_scale_2_dark_100;                                ///< Receive signal amplitude in ADC incs when reference signal is switched off (Signal 2, Dark 100%)
      uint16_t sick_reference_scale_1_dark_66;                                 ///< Receive signal amplitude in ADC incs when reference signal is switched off (Signal 1, Dark 66%)
      uint16_t sick_reference_scale_2_dark_66;                                 ///< Receive signal amplitude in ADC incs when reference signal is switched off (Signal 2, Dark 66%)
      uint16_t sick_signal_amplitude;                                          ///< Laser power in % of calibration value
      uint16_t sick_current_angle;                                             ///< Angle used for power measurement 
      uint16_t sick_peak_threshold;                                            ///< Peak threshold in ADC incs for power measurement
      uint16_t sick_angle_of_measurement;                                      ///< Angles used to reference target for power measurement
      uint16_t sick_signal_amplitude_calibration_val;                          ///< Calibration of the laser power
      uint16_t sick_stop_threshold_target_value;                               ///< Target value of the stop threshold in ADC incs
      uint16_t sick_peak_threshold_target_value;                               ///< Target value of the peak threshold in ADC incs
      uint16_t sick_stop_threshold_actual_value;                               ///< Actual value of the stop threshold in ADC incs
      uint16_t sick_peak_threshold_actual_value;                               ///< Actual value of the peak threshold in ADC incs
      uint16_t sick_reference_target_single_measured_vals;                     ///< Reference target "single measured values." Low byte: Current number of filtered single measured values. High byte: Max num filtered single measured value since power-on. 
      uint16_t sick_reference_target_mean_measured_vals;                       ///< Reference target "mean measured values." Low byte: Current number of filtered mean measured values. High byte: Max num filtered mean measured value since power-on. 
    } sick_lms_2xx_signal_status_t;
    
    /*!
     * \struct sick_lms_2xx_field_status_tag
     * \brief A structure for aggregating the data that
     *        collectively define the signal config and
     *        status.
     */
    /*!
     * \typedef sick_lms_2xx_field_status_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_field_status_tag {
      uint8_t sick_field_evaluation_number;                                    ///< Number of evaluations when the field is infirnged (lies in [1,125])
      uint8_t sick_field_set_number;                                           ///< Active field set number
      uint8_t sick_multiple_evaluation_offset_field_2;                         ///< Offset for multiple evaluation of field set 2 (see page 105 of telegram listing)
    } sick_lms_2xx_field_status_t;
    
    /*!
     * \struct sick_lms_2xx_baud_status_tag
     * \brief A structure for aggregating the data that
     *        collectively define the baud config.
     */
    /*!
     * \typedef sick_lms_2xx_baud_status_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_baud_status_tag {
      uint16_t sick_baud_rate;                                                 ///< Sick baud as reported by the device 
      uint8_t sick_permanent_baud_rate;                                        ///< 0 - When power is switched on baud rate is 9600/1 - configured transmission rate is used                                   
    } sick_lms_2xx_baud_status_t;
    
    /*!
     * \struct sick_lms_2xx_device_config_tag
     * \brief A structure for aggregating the data that
     *        collectively defines the Sick's config.
     */
    /*!
     * \typedef sick_lms_2xx_device_config_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_device_config_tag {
      uint16_t sick_blanking;                                                  ///< Maximum diameter of objects that are not to be detected (units cm)                
      uint16_t sick_fields_b_c_restart_times;                                  ///< Restart times for fields B and C
      uint16_t sick_dazzling_multiple_evaluation;                              ///< Number of scans that take place before LMS switches the outputs (only applies to availability level 1)
      uint8_t sick_peak_threshold;                                             ///< Peak threshold/black correction (This applies to Sick LMS 200/220 models, when Sick LMS 211/221/291 models are used, this value is sensitivity)
      uint8_t sick_stop_threshold;                                             ///< Stop threshold in mV (This only applies to Sick LMS 200/220 models)
      uint8_t sick_availability_level;                                         ///< Availability level of the Sick LMS     
      uint8_t sick_measuring_mode;                                             ///< Measuring mode of the device 
      uint8_t sick_measuring_units;                                            ///< Measured value and field value units
      uint8_t sick_temporary_field;                                            ///< Indicates whether the temporary field is being used
      uint8_t sick_subtractive_fields;                                         ///< Indicates whether fields A and B are subtractive
      uint8_t sick_multiple_evaluation;                                        ///< Multiple evalutation of scan data
      uint8_t sick_restart;                                                    ///< Indicates the restart level of the device
      uint8_t sick_restart_time;                                               ///< Inidicates the restart time of the device
      uint8_t sick_multiple_evaluation_suppressed_objects;                     ///< Multiple evaluation for objects less than the blanking size
      uint8_t sick_contour_a_reference;                                        ///< Contour function A
      uint8_t sick_contour_a_positive_tolerance_band;                          ///< When contour function is active the positive tolerance is defined in (cm)
      uint8_t sick_contour_a_negative_tolerance_band;                          ///< When contour function is active the negative tolerance is defined in (cm)
      uint8_t sick_contour_a_start_angle;                                      ///< When contour function is active the start angle of area to be monitored is defined (deg)
      uint8_t sick_contour_a_stop_angle;                                       ///< When contour function is active the stop angle of area to be monitored is defined (deg)
      uint8_t sick_contour_b_reference;                                        ///< Contour function B
      uint8_t sick_contour_b_positive_tolerance_band;                          ///< When contour function is active the positive tolerance is defined in (cm)
      uint8_t sick_contour_b_negative_tolerance_band;                          ///< When contour function is active the negative tolerance is defined in (cm)    uint8_t sick_contour_b_start_angle;                                       ///< When contour function is active the start angle of area to be monitored is defined (deg)
      uint8_t sick_contour_b_start_angle;                                      ///< When contour function is active the start angle of area to be monitored is defined (deg)
      uint8_t sick_contour_b_stop_angle;                                       ///< When contour function is active the stop angle of area to be monitored is defined (deg)
      uint8_t sick_contour_c_reference;                                        ///< Contour function C
      uint8_t sick_contour_c_positive_tolerance_band;                          ///< When contour function is active the positive tolerance is defined in (cm)
      uint8_t sick_contour_c_negative_tolerance_band;                          ///< When contour function is active the negative tolerance is defined in (cm)
      uint8_t sick_contour_c_start_angle;                                      ///< When contour function is active the start angle of area to be monitored is defined (deg)
      uint8_t sick_contour_c_stop_angle;                                       ///< When contour function is active the stop angle of area to be monitored is defined (deg)
      uint8_t sick_pixel_oriented_evaluation;                                  ///< Pixel oriented evaluation
      uint8_t sick_single_measured_value_evaluation_mode;                      ///< Multiple evaluation (min: 1, max: 125)
    } sick_lms_2xx_device_config_t;
    
    /*!
     * \struct sick_lms_2xx_scan_profile_b0_tag
     * \brief A structure for aggregating the data that
     *        define a scan profile obtained from reply
     *        B0 (See page 49 Telegram listing)
     */
    /*!
     * \typedef sick_lms_2xx_scan_profile_b0_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_scan_profile_b0_tag {
      uint16_t sick_num_measurements;                                          ///< Number of measurements
      uint16_t sick_measurements[SICK_MAX_NUM_MEASUREMENTS];                   ///< Range/reflectivity measurement buffer
      uint8_t sick_field_a_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field A bit value returned w/ range measurement
      uint8_t sick_field_b_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field B but value returned w/ range measurement
      uint8_t sick_field_c_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field C (or dazzle - depending upon sensor mode) value returned w/ range measurement
      uint8_t sick_telegram_index;                                             ///< Telegram index modulo 256
      uint8_t sick_real_time_scan_index;                                       ///< If real-time scan indices are requested, this value is set (modulo 256)
      uint8_t sick_partial_scan_index;                                         ///< Indicates the start angle of the scan (This is useful for partial scans)
    } sick_lms_2xx_scan_profile_b0_t;
    
    /*!
     * \struct sick_lms_2xx_scan_profile_b6_tag
     * \brief A structure for aggregating the data that
     *        define a scan profile obtained from reply
     *        B6 (See page 61 Telegram listing)
     */
    /*!
     * \typedef sick_lms_2xx_scan_profile_b6_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_scan_profile_b6_tag {
      uint16_t sick_num_measurements;                                          ///< Number of measurements
      uint16_t sick_measurements[SICK_MAX_NUM_MEASUREMENTS];                   ///< Range/reflectivity measurement buffer
      uint8_t sick_sample_size;                                                ///< Number of scans used in computing the returned mean
      uint8_t sick_telegram_index;                                             ///< Telegram index modulo 256
      uint8_t sick_real_time_scan_index;                                       ///< If real-time scan indices are requested, this value is set (modulo 256)
    } sick_lms_2xx_scan_profile_b6_t;
    
    /*!
     * \struct sick_lms_2xx_scan_profile_b7_tag
     * \brief A structure for aggregating the data that
     *        define a scan profile obtained from reply
     *        B7 (See page 63 Telegram listing)
     */
    /*!
     * \typedef sick_lms_2xx_scan_profile_b7_t
     * \brief Adopt c-style convention
     */  
    typedef struct sick_lms_2xx_scan_profile_b7_tag {
      uint16_t sick_subrange_start_index;                                      ///< Measurement subrange start index
      uint16_t sick_subrange_stop_index;                                       ///< Measurement subrange stop index
      uint16_t sick_num_measurements;                                          ///< Number of measurements
      uint16_t sick_measurements[SICK_MAX_NUM_MEASUREMENTS];                   ///< Range/reflectivity measurement buffer
      uint8_t sick_field_a_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field A bit value returned w/ range measurement
      uint8_t sick_field_b_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field B but value returned w/ range measurement
      uint8_t sick_field_c_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field C (or dazzle - depending upon sensor mode) value returned w/ range measurement
      uint8_t sick_telegram_index;                                             ///< Telegram index modulo 256
      uint8_t sick_real_time_scan_index;                                       ///< If real-time scan indices are requested, this value is set (modulo 256)
      uint8_t sick_partial_scan_index;                                         ///< Indicates the start angle of the scan (This is useful for partial scans)
    } sick_lms_2xx_scan_profile_b7_t;
    
    /*!
     * \struct sick_lms_2xx_scan_profile_bf_tag
     * \brief A structure for aggregating the data that
     *        define a scan profile obtained from reply
     *        BF (See page 71 Telegram listing)
     */
    /*!
     * \typedef sick_lms_2xx_scan_profile_bf_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_scan_profile_bf_tag {
      uint16_t sick_subrange_start_index;                                      ///< Measurement subrange start index
      uint16_t sick_subrange_stop_index;                                       ///< Measurement subrange stop index
      uint16_t sick_num_measurements;                                          ///< Number of measurements
      uint16_t sick_measurements[SICK_MAX_NUM_MEASUREMENTS];                   ///< Range/reflectivity measurement buffer
      uint8_t sick_sample_size;                                                ///< Number of scans used in computing the returned mean
      uint8_t sick_telegram_index;                                             ///< Telegram index modulo 256
      uint8_t sick_real_time_scan_index;                                       ///< If real-time scan indices are requested, this value is set (modulo 256)
    } sick_lms_2xx_scan_profile_bf_t;
    
    /*!
     * \struct sick_lms_2xx_scan_profile_c4_tag
     * \brief A structure for aggregating the data that
     *        define a scan profile obtained from reply
     *        B4 (See page 79 Telegram listing)
     */
    /*!
     * \typedef sick_lms_2xx_scan_profile_c4_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_2xx_scan_profile_c4_tag {
      uint16_t sick_num_range_measurements;                                    ///< Number of range measurements
      uint16_t sick_num_reflect_measurements;                                  ///< Number of reflectivity measurements
      uint16_t sick_range_measurements[SICK_MAX_NUM_MEASUREMENTS];             ///< Range measurement buffer
      uint16_t sick_reflect_measurements[SICK_MAX_NUM_MEASUREMENTS];           ///< Reflect measurements buffer
      uint16_t sick_reflect_subrange_start_index;                              ///< Start index of the measured reflectivity value subrange
      uint16_t sick_reflect_subrange_stop_index;                               ///< Stop index of the measured reflectivity value subrange
      uint8_t sick_field_a_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field A bit value returned w/ range measurement
      uint8_t sick_field_b_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field B but value returned w/ range measurement
      uint8_t sick_field_c_values[SICK_MAX_NUM_MEASUREMENTS];                  ///< Reflects the Field C (or dazzle - depending upon sensor mode) value returned w/ range measurement
      uint8_t sick_telegram_index;                                             ///< Telegram index modulo 256
      uint8_t sick_real_time_scan_index;                                       ///< If real-time scan indices are requested, this value is set (modulo 256)
    } sick_lms_2xx_scan_profile_c4_t;
    
    /** Constructor */
    SickLMS2xx( const std::string sick_device_path );
    
    /** Destructor */
    ~SickLMS2xx( );
    
    /** Initializes the Sick */
    void Initialize( const sick_lms_2xx_baud_t desired_baud_rate, const uint32_t delay = 0 )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Uninitializes the Sick */
    void Uninitialize( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Gets the Sick LMS 2xx device path */
    std::string GetSickDevicePath( ) const;
    
    /** Gets the Sick LMS 2xx device type */
    sick_lms_2xx_type_t GetSickType( ) const throw( SickConfigException );

    /** Gets the scan angle currently being used by the device */
    double GetSickScanAngle( ) const throw( SickConfigException );

    /** Gets the scan resolution currently being used by the device */
    double GetSickScanResolution( ) const throw( SickConfigException );

    /** Set the measurement units of the device (in EEPROM) */
    void SetSickMeasuringUnits( const sick_lms_2xx_measuring_units_t sick_units = SICK_MEASURING_UNITS_MM )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Get the current measurement units of the device */
    SickLMS2xx::sick_lms_2xx_measuring_units_t GetSickMeasuringUnits( ) const throw( SickConfigException );
    
    /** Sets the sensitivity value for the device (in EEPROM). NOTE: Only applies to LMS 211/221/291 models. */
    void SetSickSensitivity( const sick_lms_2xx_sensitivity_t sick_sensitivity = SICK_SENSITIVITY_STANDARD )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Get the current Sick LMS 2xx sensitivity level. NOTE: Only applies to LMS 211/221/291 models. */
    sick_lms_2xx_sensitivity_t GetSickSensitivity( ) const throw( SickConfigException );

    /** Sets the peak threshold mode for the device (in EEPROM). NOTE: Only applies to LMS 200/220 models */
    void SetSickPeakThreshold( const sick_lms_2xx_peak_threshold_t sick_peak_threshold = SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );
    
    /** Get the current Sick LMS 2xx sensitivity level. NOTE: Only applies to LMS 211/221/291 models. */
    sick_lms_2xx_peak_threshold_t GetSickPeakThreshold( ) const throw( SickConfigException );;
    
    /** Sets the measuring mode for the device (in EEPROM). See page 98 of the telegram listing for more details. */
    void SetSickMeasuringMode( const sick_lms_2xx_measuring_mode_t sick_measuring_mode = SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Get the current Sick LMS 2xx measuring mode */
    sick_lms_2xx_measuring_mode_t GetSickMeasuringMode( ) const throw( SickConfigException );

    /** Get the current Sick LMS 2xx operating mode */
    sick_lms_2xx_operating_mode_t GetSickOperatingMode( ) const throw( SickConfigException );
    
    /** Sets the availability of the device (in EEPROM). See page 98 of the telegram listing for more details. */
    void SetSickAvailability( const uint8_t sick_availability_flags = SICK_FLAG_AVAILABILITY_DEFAULT )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Gets the current availability flags for the device */
    uint8_t GetSickAvailability( ) const throw( SickConfigException );
    
    /** Sets the variant type for the device (in EEPROM) */
    void SetSickVariant( const sick_lms_2xx_scan_angle_t scan_angle, const sick_lms_2xx_scan_resolution_t scan_resolution )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Gets measurement data from the Sick. NOTE: Data can be either range or reflectivity given the Sick mode. */
    void GetSickScan( unsigned int * const measurement_values,
		      unsigned int & num_measurement_values,
		      unsigned int * const sick_field_a_values = NULL,
		      unsigned int * const sick_field_b_values = NULL,
		      unsigned int * const sick_field_c_values = NULL,
		      unsigned int * const sick_telegram_index = NULL,
		      unsigned int * const sick_real_time_scan_index = NULL ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Gets range and reflectivity data from the Sick. NOTE: This only applies to Sick LMS 211/221/291-S14! */
    void GetSickScan( unsigned int * const range_values,
		      unsigned int * const reflect_values,
		      unsigned int & num_range_measurements,
		      unsigned int & num_reflect_measurements,
		      unsigned int * const sick_field_a_values = NULL,
		      unsigned int * const sick_field_b_values = NULL,
		      unsigned int * const sick_field_c_values = NULL,
		      unsigned int * const sick_telegram_index = NULL,
		      unsigned int * const sick_real_time_scan_index = NULL ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Gets measurement data from the Sick. NOTE: Data can be either range or reflectivity given the Sick mode. */
    void GetSickScanSubrange( const uint16_t sick_subrange_start_index,
			      const uint16_t sick_subrange_stop_index,
			      unsigned int * const measurement_values,
			      unsigned int & num_measurement_values,
			      unsigned int * const sick_field_a_values = NULL,
			      unsigned int * const sick_field_b_values = NULL,
			      unsigned int * const sick_field_c_values = NULL,
			      unsigned int * const sick_telegram_index = NULL,
			      unsigned int * const sick_real_time_scan_index = NULL ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);
    
    /** Gets partial scan measurements from the Sick LMS 2xx. NOTE: Data can be either range or reflectivity depending upon the given Sick mode. */
    void GetSickPartialScan( unsigned int * const measurement_values,
			     unsigned int & num_measurement_values,
			     unsigned int & partial_scan_index,
			     unsigned int * const sick_field_a_values = NULL,
			     unsigned int * const sick_field_b_values = NULL,
			     unsigned int * const sick_field_c_values = NULL,
			     unsigned int * const sick_telegram_index = NULL,
			     unsigned int * const sick_real_time_scan_index = NULL ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Gets mean measured values from the Sick LMS */
    void GetSickMeanValues( const uint8_t sick_sample_size,
			    unsigned int * const measurement_values,
			    unsigned int & num_measurement_values,
			    unsigned int * const sick_telegram_index = NULL,
			    unsigned int * const sick_real_time_index = NULL ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Gets mean measured values from the Sick LMS */
    void GetSickMeanValuesSubrange( const uint8_t sick_sample_size,
				    const uint16_t sick_subrange_start_index,
				    const uint16_t sick_subrange_stop_index,
				    unsigned int * const measurement_values,
				    unsigned int & num_measurement_values,
				    unsigned int * const sick_telegram_index = NULL,
				    unsigned int * const sick_real_time_index = NULL ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Acquire the Sick LMS status */
    sick_lms_2xx_status_t GetSickStatus( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );

    /** Indicates whether the Sick is an LMS Fast */
    bool IsSickLMS2xxFast( ) const throw( SickConfigException );
    
    /** Resets Sick LMS field values */
    void ResetSick( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );
    
    /** Get Sick status as a string */
    std::string GetSickStatusAsString( ) const;
    
    /** Get Sick software info as a string */
    std::string GetSickSoftwareVersionAsString( ) const;
    
    /** Get Sick config as a string */
    std::string GetSickConfigAsString( ) const;
    
    /** Print the Sick LMS status */
    void PrintSickStatus( ) const;

    /** Print the Sick LMS software versions */
    void PrintSickSoftwareVersion( ) const;

    /** Print the Sick LMS configuration */
    void PrintSickConfig( ) const;

    /*
     * NOTE: The following methods are given to make working with our
     *       predefined types a bit more manageable.
     */

    /** Converts the LMS's type to a corresponding string */
    static std::string SickTypeToString( const sick_lms_2xx_type_t sick_type );

    /** A utility function for converting integers to lms_sick_scan_angle_t */
    static sick_lms_2xx_scan_angle_t IntToSickScanAngle( const int scan_angle_int );

    /** A utility function for converting ints to lms_sick_scan_resolution_t */
    static sick_lms_2xx_scan_resolution_t IntToSickScanResolution( const int scan_resolution_int );
    
    /** A utility function for converting doubles to lms_sick_scan_resolution_t */
    static sick_lms_2xx_scan_resolution_t DoubleToSickScanResolution( const double scan_resolution_double );
    
    /** Converts the given bad, returns a string representing that baud rate. */
    static std::string SickBaudToString( const sick_lms_2xx_baud_t baud_rate );

    /** A utility function for converting integers to lms_baud_t */
    static sick_lms_2xx_baud_t IntToSickBaud( const int baud_int );
    
    /** A utility function for converting baud strings to lms_baud_t */
    static sick_lms_2xx_baud_t StringToSickBaud( const std::string baud_str );

    /** Converts the LMS's status to a corresponding string */
    static std::string SickStatusToString( const sick_lms_2xx_status_t sick_status );
    
    /** Converts the LMS's measuring mode to a corresponding string */
    static std::string SickMeasuringModeToString( const sick_lms_2xx_measuring_mode_t sick_measuring_mode );

    /** Converts the LMS's measuring mode to a corresponding string */
    static std::string SickOperatingModeToString( const sick_lms_2xx_operating_mode_t sick_operating_mode );
    
    /** Converts the LMS's sensitivity to string */
    static std::string SickSensitivityToString( const sick_lms_2xx_sensitivity_t sick_sensitivity );

    /** Converts the LMS's peak threshold to string */
    static std::string SickPeakThresholdToString( const sick_lms_2xx_peak_threshold_t sick_peak_threshold );
    
    /** Converts the LMS's measuring units to a corresponding string */
    static std::string SickMeasuringUnitsToString( const sick_lms_2xx_measuring_units_t sick_units );    

  protected:

    /** A path to the device at which the sick can be accessed. */
    std::string _sick_device_path;

    /** The baud rate at which to communicate with the Sick */
    sick_lms_2xx_baud_t _curr_session_baud;

    /** The desired baud rate for communicating w/ the Sick */
    sick_lms_2xx_baud_t _desired_session_baud;
    
    /** A string representing the type of device */
    sick_lms_2xx_type_t _sick_type;

    /** The operating parameters of the device */
    sick_lms_2xx_operating_status_t _sick_operating_status;

    /** The current software version being run on the device */
    sick_lms_2xx_software_status_t _sick_software_status;

    /** The restart configuration of the device */
    sick_lms_2xx_restart_status_t _sick_restart_status;

    /** The pollution measurement status */
    sick_lms_2xx_pollution_status_t _sick_pollution_status;

    /** The signal status of the device */
    sick_lms_2xx_signal_status_t _sick_signal_status;

    /** The field configuration for the device */
    sick_lms_2xx_field_status_t _sick_field_status;

    /** The baud configuration of the device */
    sick_lms_2xx_baud_status_t _sick_baud_status;

    /** The device configuration for the Sick */
    sick_lms_2xx_device_config_t _sick_device_config;

    /** Used when the device is streaming mean values */
    uint8_t _sick_mean_value_sample_size;

    /** Used when the device is streaming a scan subrange */
    uint16_t _sick_values_subrange_start_index;

    /** Used when the device is streaming a scan subrange */
    uint16_t _sick_values_subrange_stop_index;
    
    /** Stores information about the original terminal settings */
    struct termios _old_term;

    /** Opens the terminal for serial communication. */
    void _setupConnection() throw( SickIOException, SickThreadException );
    void _setupConnection(const uint32_t delay ) throw( SickIOException, SickThreadException );

    /** Closes the serial communication terminal. */
    void _teardownConnection( ) throw( SickIOException );

    /** Sends a message to the LMS and get the expected reply using th 0x80 rule.   @todo Check difference in comments? */
    void _sendMessageAndGetReply( const SickLMS2xxMessage &sick_send_message,
				  SickLMS2xxMessage &sick_recv_message,
				  const unsigned int timeout_value,
				  const unsigned int num_tries ) throw( SickIOException, SickThreadException, SickTimeoutException ); 

    /** Sends a message to the LMS and get the expected reply using th 0x80 rule. @todo Check difference in comments? */
    void _sendMessageAndGetReply( const SickLMS2xxMessage &sick_send_message,
				  SickLMS2xxMessage &sick_recv_message,
				  const uint8_t reply_code,
				  const unsigned int timeout_value,
				  const unsigned int num_tries ) throw( SickIOException, SickThreadException, SickTimeoutException ); 
    
    /** Flushes the terminal I/O buffers */
    void _flushTerminalBuffer( ) throw ( SickThreadException );
    
    /** Sets the baud rate for communication with the LMS. */
    void _setSessionBaud( const sick_lms_2xx_baud_t baud_rate ) throw( SickIOException, SickThreadException, SickTimeoutException );

    /** Tests communication wit the LMS at a particular baud rate. */
    bool _testSickBaud( const sick_lms_2xx_baud_t baud_rate ) throw( SickIOException, SickThreadException );

    /** Changes the terminal's baud rate. */
    void _setTerminalBaud( const sick_lms_2xx_baud_t sick_baud ) throw( SickIOException, SickThreadException );

    /** Gets the type of Sick LMS */
    void _getSickType( ) throw( SickTimeoutException, SickIOException, SickThreadException );

    /** Gets the current Sick configuration settings */
    void _getSickConfig( ) throw( SickTimeoutException, SickIOException, SickThreadException );

    /** Sets the Sick configuration in flash */
    void _setSickConfig( const sick_lms_2xx_device_config_t &sick_config ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException );
    
    /** Gets the status of the LMS */
    void _getSickStatus( ) throw( SickTimeoutException, SickIOException, SickThreadException );

    /** Gets the error status of the Sick LMS */
    void _getSickErrors( unsigned int * const num_sick_errors = NULL,
			 uint8_t * const error_type_buffer = NULL,
			 uint8_t * const error_num_buffer = NULL ) throw( SickTimeoutException, SickIOException, SickThreadException );

    /** Switch Sick LMS to installation mode */
    void _setSickOpModeInstallation( )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to diagnostic mode */
    void _setSickOpModeDiagnostic( )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (request range data) */
    void _setSickOpModeMonitorRequestValues( )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (stream range) */
    void _setSickOpModeMonitorStreamValues( )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (stream range and reflectivity) */
    void _setSickOpModeMonitorStreamRangeAndReflectivity( )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (stream range from a partial scan) */
    void _setSickOpModeMonitorStreamValuesFromPartialScan( )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (stream mean measured values) */
    void _setSickOpModeMonitorStreamMeanValues( const uint8_t sample_size )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (stream mean measured values) */
    void _setSickOpModeMonitorStreamValuesSubrange( const uint16_t subrange_start_index, const uint16_t subrange_stop_index )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);

    /** Switch Sick LMS to monitor mode (stream mean measured values subrange) */
    void _setSickOpModeMonitorStreamMeanValuesSubrange( const uint16_t sample_size, const uint16_t subrange_start_index, const uint16_t subrange_stop_index )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);
    
    /** Switches the operating mode of the LMS. */
    void _switchSickOperatingMode( const uint8_t sick_mode, const uint8_t * const mode_params = NULL )
      throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException);
    
    /** Parses the scan profile returned w/ message B0 */
    void _parseSickScanProfileB0( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_b0_t &sick_scan_profile ) const;

    /** Parses the scan profile returned w/ message B6 */
    void _parseSickScanProfileB6( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_b6_t &sick_scan_profile ) const;

    /** Parses the scan profile returned w/ message B6 */
    void _parseSickScanProfileB7( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_b7_t &sick_scan_profile ) const;

    /** Parses the scan profile returned w/ message BF */
    void _parseSickScanProfileBF( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_bf_t &sick_scan_profile ) const;
    
    /** Parses the scan profile returned w/ message C4 */
    void _parseSickScanProfileC4( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_c4_t &sick_scan_profile ) const;

    /** A function for parsing a byte sequence into a device config structure */
    void _parseSickConfigProfile( const uint8_t * const src_buffer, sick_lms_2xx_device_config_t &sick_device_config ) const;

    /** Acquires the bit mask to extract the field bit values returned with each range measurement */
    void _extractSickMeasurementValues( const uint8_t * const byte_sequence, const uint16_t num_measurements, uint16_t * const measured_values,
					uint8_t * const field_a_values = NULL, uint8_t * const field_b_values = NULL, uint8_t * const field_c_values = NULL ) const;
    
    /** Tells whether the device is returning real-time indices */
    bool _returningRealTimeIndices( ) const { return _sick_device_config.sick_availability_level & SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES; }

    /** Indicates whether the given unit value is defined */
    bool _validSickMeasuringUnits( const sick_lms_2xx_measuring_units_t sick_units ) const;

    /** Indicates whether the given scan angle is defined */
    bool _validSickScanAngle( const sick_lms_2xx_scan_angle_t sick_scan_angle ) const;

    /** Indicates whether the given scan resolution is defined */
    bool _validSickScanResolution( const sick_lms_2xx_scan_resolution_t sick_scan_resolution ) const;

    /** Indicates whether the given sensitivity is defined */
    bool _validSickSensitivity( const sick_lms_2xx_sensitivity_t sick_sensitivity ) const;

    /** Indicates whether the given peak threshold is defined */
    bool _validSickPeakThreshold( const sick_lms_2xx_peak_threshold_t sick_peak_threshold ) const;
    
    /** Indicates whether the given sensitivity is defined */
    bool _validSickMeasuringMode( const sick_lms_2xx_measuring_mode_t sick_measuring_mode ) const;

    /** Indicates whether the Sick LMS is type 200 */
    bool _isSickLMS200( ) const;
    
    /** Indicates whether the Sick LMS is type 211 */
    bool _isSickLMS211( ) const;
    
    /** Indicates whether the Sick LMS is type 220 */
    bool _isSickLMS220( ) const;
    
    /** Indicates whether the Sick LMS is type 221 */
    bool _isSickLMS221( ) const;
    
    /** Indicates whether the Sick LMS is type 291 */
    bool _isSickLMS291( ) const;

    /** Indicates whether the Sick LMS type is unknown */
    bool _isSickUnknown( ) const;
    
    /** Given a baud rate as an integer, gets a LMS baud rate command. */
    sick_lms_2xx_baud_t _baudToSickBaud( const int baud_rate ) const;

    /** Given a bytecode representing Sick LMS availability, returns a corresponding string */
    std::string _sickAvailabilityToString( const uint8_t availability_code ) const;

    /** Given a bytecode representing Sick LMS restart mode, returns a corresponding string */
    std::string _sickRestartToString( const uint8_t restart_code ) const;
    
    /** Converts the LMS's temporary field value to a string */
    std::string _sickTemporaryFieldToString( const uint8_t temp_field_code ) const;

    /** Converts the LMS's subtractive field value to a string */
    std::string _sickSubtractiveFieldsToString( const uint8_t subt_field_code ) const;

    /** Converts the LMS's contour function status code to a string */
    std::string _sickContourFunctionToString( const uint8_t contour_function_code ) const;
    
    /** Converts the LMS's variant to a corresponding string */
    std::string _sickVariantToString( const unsigned int sick_variant ) const;

  };

  /*!
   * \typedef sick_lms_2xx_type_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_type_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_type_t sick_lms_2xx_type_t;

  /*!
   * \typedef sick_lms_2xx_variant_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_variant_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_variant_t sick_lms_2xx_variant_t;

  /*!
   * \typedef sick_lms_2xx_scan_angle_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_scan_angle_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_scan_angle_t sick_lms_2xx_scan_angle_t;

  /*!
   * \typedef sick_lms_2xx_scan_resolution_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_scan_resolution_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_scan_resolution_t sick_lms_2xx_scan_resolution_t;

  /*!
   * \typedef sick_lms_2xx_measuring_units_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_measuring_units_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_measuring_units_t sick_lms_2xx_measuring_units_t;

  /*!
   * \typedef sick_lms_2xx_sensitivity_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_sensitivity_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_sensitivity_t sick_lms_2xx_sensitivity_t;

  /*!
   * \typedef sick_lms_2xx_peak_threshold_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_peak_threshold_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_peak_threshold_t sick_lms_2xx_peak_threshold_t;
  
  /*!
   * \typedef sick_lms_2xx_status_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_status_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_status_t sick_lms_2xx_status_t;

  /*!
   * \typedef sick_lms_2xx_measuring_mode_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_measuring_mode_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_measuring_mode_t sick_lms_2xx_measuring_mode_t;

  /*!
   * \typedef sick_lms_2xx_operating_mode_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_operating_mode_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_operating_mode_t sick_lms_2xx_operating_mode_t;

  /*!
   * \typedef sick_lms_2xx_baud_t
   * \brief Makes working w/ SickLMS2xx::sick_lms_2xx_baud_t a bit easier
   */
  typedef SickLMS2xx::sick_lms_2xx_baud_t sick_lms_2xx_baud_t;
  
} //namespace SickToolbox
  
#endif //SICK_LMS_2XX_HH
