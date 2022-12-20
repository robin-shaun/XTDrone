/*!
 * \file SickLD.hh
 * \brief Defines the SickLD class for working with the
 *        Sick LD-OEM/LD-LRS long range LIDARs.
 *
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

#ifndef SICK_LD_HH
#define SICK_LD_HH

/* Macros */
#define DEFAULT_SICK_IP_ADDRESS                          "192.168.1.10"  ///< Default Sick LD INet 4 address
#define DEFAULT_SICK_TCP_PORT                                   (49152)  ///< Default TCP port
#define DEFAULT_SICK_MESSAGE_TIMEOUT                (unsigned int)(5e6)  ///< The max time to wait for a message reply (usecs)
#define DEFAULT_SICK_CONNECT_TIMEOUT                (unsigned int)(1e6)  ///< The max time to wait before considering a connection attempt as failed (usecs)
#define DEFAULT_SICK_NUM_SCAN_PROFILES                              (0)  ///< Setting this value to 0 will tell the Sick LD to stream measurements when measurement data is requested (NOTE: A profile is a single scans worth of range measurements)
#define DEFAULT_SICK_SIGNAL_SET                                     (0)  ///< Default Sick signal configuration

/**
 * \def SWAP_VALUES(x,y,t)
 * \brief A simple macro for swapping two values.
 */
#define SWAP_VALUES(x,y,t) (t=x,x=y,y=t);

/* Definition dependencies */
#include <string>
#include <vector>
#include <pthread.h>
#include <arpa/inet.h>

#include "SickLIDAR.hh"
#include "SickLDBufferMonitor.hh"
#include "SickLDMessage.hh"
#include "SickException.hh"

/**
 * \namespace SickToolbox
 * \brief Encapsulates the Sick LIDAR Matlab/C++ toolbox
 */
namespace SickToolbox {

  /**
   * \class SickLD
   * \brief Provides a simple driver interface for working with the
   *        Sick LD-OEM/Sick LD-LRS long-range models via Ethernet.
   */
  class SickLD : public SickLIDAR< SickLDBufferMonitor, SickLDMessage > {

  public:

    /* Some constants for the developer/end-user */
    static const uint16_t SICK_MAX_NUM_MEASUREMENTS = 2881;                             ///< Maximum number of measurements per sector
    static const uint16_t SICK_MAX_NUM_SECTORS = 8;                                     ///< Maximum number of scan sectors (NOTE: This value must be even)
    static const uint16_t SICK_MAX_NUM_MEASURING_SECTORS = 4;                           ///< Maximum number of active/measuring scan sectors
    static const uint16_t SICK_MAX_SCAN_AREA = 360;                                     ///< Maximum area that can be covered in a single scan (deg)
    static const uint16_t SICK_MIN_MOTOR_SPEED = 5;                                     ///< Minimum motor speed in Hz
    static const uint16_t SICK_MAX_MOTOR_SPEED = 20;                                    ///< Maximum motor speed in Hz
    static const uint16_t SICK_MIN_VALID_SENSOR_ID = 1;                                 ///< The lowest value the Sick will accept as a Sensor ID
    static const uint16_t SICK_MAX_VALID_SENSOR_ID = 254;                               ///< The largest value the Sick will accept as a Sensor ID    
    static const uint16_t SICK_MAX_MEAN_PULSE_FREQUENCY = 10800;                        ///< Max mean pulse frequence of the current device configuration (in Hz) (see page 22 of the operator's manual)
    static const uint16_t SICK_MAX_PULSE_FREQUENCY = 14400;                             ///< Max pulse frequency of the device (in Hz) (see page 22 of the operator's manual)
    static const uint16_t SICK_NUM_TICKS_PER_MOTOR_REV = 5760;                          ///< Odometer ticks per revolution of the Sick LD scan head
    static constexpr double SICK_MAX_SCAN_ANGULAR_RESOLUTION = 0.125;                       ///< Minimum valid separation between laser pulses in active scan ares (deg)
    static constexpr double SICK_DEGREES_PER_MOTOR_STEP = 0.0625;                           ///< Each odometer tick is equivalent to rotating the scan head this many degrees
    
    /* Sick LD sensor modes of operation */
    static const uint8_t SICK_SENSOR_MODE_IDLE = 0x01;                                  ///< The Sick LD is powered but idle
    static const uint8_t SICK_SENSOR_MODE_ROTATE = 0x02;                                ///< The Sick LD prism is rotating, but laser is off
    static const uint8_t SICK_SENSOR_MODE_MEASURE = 0x03;                               ///< The Sick LD prism is rotating, and the laser is on
    static const uint8_t SICK_SENSOR_MODE_ERROR = 0x04;                                 ///< The Sick LD is in error mode
    static const uint8_t SICK_SENSOR_MODE_UNKNOWN = 0xFF;                               ///< The Sick LD is in an unknown state
  
    /* Sick LD motor modes */
    static const uint8_t SICK_MOTOR_MODE_OK = 0x00;                                     ///< Motor is functioning properly
    static const uint8_t SICK_MOTOR_MODE_SPIN_TOO_HIGH = 0x09;                          ///< Motor spin too low (i.e. rotational velocity too low)
    static const uint8_t SICK_MOTOR_MODE_SPIN_TOO_LOW = 0x04;                           ///< Motor spin too high (i.e. rotational velocity too fast)
    static const uint8_t SICK_MOTOR_MODE_ERROR = 0x0B;                                  ///< Motor stops or coder error
    static const uint8_t SICK_MOTOR_MODE_UNKNOWN = 0xFF;                                ///< Motor is in an unknown state
  
    /* Sick LD service codes */
    static const uint8_t SICK_STAT_SERV_CODE = 0x01;                                    ///< Status service code 
    static const uint8_t SICK_CONF_SERV_CODE = 0x02;                                    ///< Configuration service code
    static const uint8_t SICK_MEAS_SERV_CODE = 0x03;                                    ///< Measurement service code
    static const uint8_t SICK_WORK_SERV_CODE = 0x04;                                    ///< Working service code
    static const uint8_t SICK_ROUT_SERV_CODE = 0x06;                                    ///< Routing service code
    static const uint8_t SICK_FILE_SERV_CODE = 0x07;                                    ///< File service code 
    static const uint8_t SICK_MONR_SERV_CODE = 0x08;                                    ///< Monitor service code 
  
    /* Sick LD status services (service code 0x01) */
    static const uint8_t SICK_STAT_SERV_GET_ID = 0x01;                                  ///< Request the Sick LD ID
    static const uint8_t SICK_STAT_SERV_GET_STATUS = 0x02;                              ///< Request status information
    static const uint8_t SICK_STAT_SERV_GET_SIGNAL = 0x04;                              ///< Reads the value of the switch and LED port
    static const uint8_t SICK_STAT_SERV_SET_SIGNAL = 0x05;                              ///< Sets the switches and LEDs
    static const uint8_t SICK_STAT_SERV_LD_REGISTER_APPLICATION = 0x06;                 ///< Registers the ID data for the application firmware

    /* Sick LD status service GET_IDENTIFICATION request codes */
    static const uint8_t SICK_STAT_SERV_GET_ID_SENSOR_PART_NUM = 0x00;                  ///< Request the sensor's part number
    static const uint8_t SICK_STAT_SERV_GET_ID_SENSOR_NAME = 0x01;                      ///< Request the sensor's name
    static const uint8_t SICK_STAT_SERV_GET_ID_SENSOR_VERSION = 0x02;                   ///< Request the sensor's version
    static const uint8_t SICK_STAT_SERV_GET_ID_SENSOR_SERIAL_NUM = 0x03;                ///< Request the sensor's serial number
    static const uint8_t SICK_STAT_SERV_GET_ID_SENSOR_EDM_SERIAL_NUM = 0x04;            ///< Request the edm??? serial number
    static const uint8_t SICK_STAT_SERV_GET_ID_FIRMWARE_PART_NUM = 0x10;                ///< Requess the firmware's part number
    static const uint8_t SICK_STAT_SERV_GET_ID_FIRMWARE_NAME = 0x11;                    ///< Request the firmware's name
    static const uint8_t SICK_STAT_SERV_GET_ID_FIRMWARE_VERSION = 0x12;                 ///< Request the firmware's version
    static const uint8_t SICK_STAT_SERV_GET_ID_APP_PART_NUM = 0x20;                     ///< Request the application part number
    static const uint8_t SICK_STAT_SERV_GET_ID_APP_NAME = 0x21;                         ///< Request the application name
    static const uint8_t SICK_STAT_SERV_GET_ID_APP_VERSION = 0x22;                      ///< Request the application version
  
    /* Sick LD configuration services (service code 0x02) */
    static const uint8_t SICK_CONF_SERV_SET_CONFIGURATION = 0x01;                       ///< Set the Sick LD configuration
    static const uint8_t SICK_CONF_SERV_GET_CONFIGURATION = 0x02;                       ///< Read the Sick LD configuration information
    static const uint8_t SICK_CONF_SERV_SET_TIME_ABSOLUTE = 0x03;                       ///< Set the internal clock to a timestamp value
    static const uint8_t SICK_CONF_SERV_SET_TIME_RELATIVE = 0x04;                       ///< Correct the internal clock by some value
    static const uint8_t SICK_CONF_SERV_GET_SYNC_CLOCK = 0x05;                          ///< Read the internal time of the LD-OEM/LD-LRS
    static const uint8_t SICK_CONF_SERV_SET_FILTER = 0x09;                              ///< Set the filter configuration
    static const uint8_t SICK_CONF_SERV_SET_FUNCTION = 0x0A;                            ///< Assigns a measurement function to an angle range
    static const uint8_t SICK_CONF_SERV_GET_FUNCTION = 0x0B;                            ///< Returns the configuration of the given sector

    /* Sick LD configuration filter codes */
    static const uint8_t SICK_CONF_SERV_SET_FILTER_NEARFIELD = 0x01;                    ///< Code for identifying filter type: nearfield suppression
  
    /* Sick LD nearfield suppression configuration codes */
    static const uint8_t SICK_CONF_SERV_SET_FILTER_NEARFIELD_OFF = 0x00;                ///< Used to set nearfield suppression off
    static const uint8_t SICK_CONF_SERV_SET_FILTER_NEARFIELD_ON = 0x01;                 ///< Used to set nearfield suppression on
  
    /* Sick LD measurement services (service code 0x03) */
    static const uint8_t SICK_MEAS_SERV_GET_PROFILE = 0x01;                             ///< Requests n profiles of a defined format
    static const uint8_t SICK_MEAS_SERV_CANCEL_PROFILE = 0x02;                          ///< Stops profile output

    /* Sick LD working services (service code 0x04) */
    static const uint8_t SICK_WORK_SERV_RESET = 0x01;                                   ///< Sick LD enters a reset sequence
    static const uint8_t SICK_WORK_SERV_TRANS_IDLE = 0x02;                              ///< Sick LD enters IDLE mode (motor stops and laser is turned off)
    static const uint8_t SICK_WORK_SERV_TRANS_ROTATE = 0x03;                            ///< Sick LD enters ROTATE mode (motor starts and rotates with a specified speed in Hz, laser is off)
    static const uint8_t SICK_WORK_SERV_TRANS_MEASURE = 0x04;                           ///< Sick LD enters MEASURE mode (laser starts with next revolution) 

    /* Sick LD working service DO_RESET request codes */
    static const uint8_t SICK_WORK_SERV_RESET_INIT_CPU = 0x00;                          ///< Sick LD does a complete reset (Reinitializes the CPU)
    static const uint8_t SICK_WORK_SERV_RESET_KEEP_CPU = 0x01;                          ///< Sick LD does a partial reset (CPU is not reinitialized)
    static const uint8_t SICK_WORK_SERV_RESET_HALT_APP = 0x02;                          ///< Sick LD does a minimal reset (Application is halted and device enters IDLE state)

    /* Sick LD working service TRANS_MEASURE return codes */
    static const uint8_t SICK_WORK_SERV_TRANS_MEASURE_RET_OK = 0x00;                    ///< Sick LD is ready to stream/obtain scan profiles
    static const uint8_t SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_MAX_PULSE = 0x01;         ///< Sick LD reports config yields a max laser pulse frequency that is too high
    static const uint8_t SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_MEAN_PULSE = 0x02;        ///< Sick LD reports config yields a max mean pulse frequency that is too high
    static const uint8_t SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_SECT_BORDER = 0x03;       ///< Sick LD reports sector borders are not configured correctly
    static const uint8_t SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_SECT_BORDER_MULT = 0x04;  ///< Sick LD reports sector borders are not a multiple of the step angle
  
    /* Sick LD interface routing services (service code 0x06) */
    static const uint8_t SICK_ROUT_SERV_COM_ATTACH = 0x01;                              ///< Attach a master (host) communications interface
    static const uint8_t SICK_ROUT_SERV_COM_DETACH = 0x02;                              ///< Detach a master (host) communications interface
    static const uint8_t SICK_ROUT_SERV_COM_INITIALIZE = 0x03;                          ///< Initialize the interface (Note: using this may not be necessary for some interfaces, e.g. Ethernet) 
    static const uint8_t SICK_ROUT_SERV_COM_OUTPUT = 0x04;                              ///< Output data to the interface
    static const uint8_t SICK_ROUT_SERV_COM_DATA = 0x05;                                ///< Forward data received on specified interface to master interface

    /* Sick LD file services (service code 0x07) */
    static const uint8_t SICK_FILE_SERV_DIR = 0x01;                                     ///< List the stored files in flash memory
    static const uint8_t SICK_FILE_SERV_SAVE = 0x02;                                    ///< Saves the data into flash memory
    static const uint8_t SICK_FILE_SERV_LOAD = 0x03;                                    ///< Recalls a file from the flash
    static const uint8_t SICK_FILE_SERV_DELETE = 0x04;                                  ///< Deletes a file from the flash

    /* Sick LD monitor services (service code 0x08) */
    static const uint8_t SICK_MONR_SERV_MONITOR_RUN = 0x01;                             ///< Enable/disable monitor services
    static const uint8_t SICK_MONR_SERV_MONITOR_PROFILE_LOG = 0x02;                     ///< Enable/disable profile logging

    /* Sick LD configuration keys */
    static const uint8_t SICK_CONF_KEY_RS232_RS422 = 0x01;                              ///< Key for configuring RS-232/RS-422
    static const uint8_t SICK_CONF_KEY_CAN = 0x02;                                      ///< Key for configuring CAN 
    static const uint8_t SICK_CONF_KEY_ETHERNET = 0x05;                                 ///< Key for configuring Ethernet
    static const uint8_t SICK_CONF_KEY_GLOBAL = 0x10;                                   ///< Key for global configuration

    /* Sick LD sector configuration codes */
    static const uint8_t SICK_CONF_SECTOR_NOT_INITIALIZED = 0x00;                       ///< Sector is uninitialized
    static const uint8_t SICK_CONF_SECTOR_NO_MEASUREMENT = 0x01;                        ///< Sector has no measurements
    static const uint8_t SICK_CONF_SECTOR_RESERVED = 0x02;                              ///< Sector is reserved by Sick LD 
    static const uint8_t SICK_CONF_SECTOR_NORMAL_MEASUREMENT = 0x03;                    ///< Sector is returning measurements
    static const uint8_t SICK_CONF_SECTOR_REFERENCE_MEASUREMENT = 0x04;                 ///< Sector can be used as reference measurement

    /* Sick LD profile formats */
    static const uint16_t SICK_SCAN_PROFILE_RANGE = 0x39FF;                             ///< Request sector scan data w/o any echo data
    /*
     * SICK_SCAN_PROFILE_RANGE format (0x39FF) interpretation: 
     * (See page 32 of telegram listing for fieldname definitions)
     *
     * Field Name   | Send
     * --------------------
     * PROFILESENT  | YES
     * PROFILECOUNT | YES
     * LAYERNUM     | YES
     * SECTORNUM    | YES
     * DIRSTEP      | YES
     * POINTNUM     | YES
     * TSTART       | YES
     * STARTDIR     | YES
     * DISTANCE-n   | YES
     * DIRECTION-n  | NO
     * ECHO-n       | NO
     * TEND         | YES
     * ENDDIR       | YES
     * SENSTAT      | YES
     */

    /* Sick LD profile formats */
    static const uint16_t SICK_SCAN_PROFILE_RANGE_AND_ECHO = 0x3DFF;                    ///< Request sector scan data w/ echo data
    /*
     * SICK_SCAN_PROFILE_RANGE format (0x3DFF) interpretation: 
     * (See page 32 of telegram listing for fieldname definitions)
     *
     * Field Name   | Send
     * --------------------
     * PROFILESENT  | YES
     * PROFILECOUNT | YES
     * LAYERNUM     | YES
     * SECTORNUM    | YES
     * DIRSTEP      | YES
     * POINTNUM     | YES
     * TSTART       | YES
     * STARTDIR     | YES
     * DISTANCE-n   | YES
     * DIRECTION-n  | NO
     * ECHO-n       | YES
     * TEND         | YES
     * ENDDIR       | YES
     * SENSTAT      | YES
     */
  
    /* Masks for working with the Sick LD signals
     *
     * NOTE: Although the Sick LD manual defines the flag
     *       values for red and green LEDs the operation
     *       of these LEDs are reserved. So they can't
     *       be set by the device driver.
     */
    static const uint8_t SICK_SIGNAL_LED_YELLOW_A = 0x01;                               ///< Mask for first yellow LED
    static const uint8_t SICK_SIGNAL_LED_YELLOW_B = 0x02;                               ///< Mask for second yellow LED
    static const uint8_t SICK_SIGNAL_LED_GREEN = 0x04;                                  ///< Mask for green LED
    static const uint8_t SICK_SIGNAL_LED_RED = 0x08;                                    ///< Mask for red LED
    static const uint8_t SICK_SIGNAL_SWITCH_0 = 0x10;                                   ///< Mask for signal switch 0
    static const uint8_t SICK_SIGNAL_SWITCH_1 = 0x20;                                   ///< Mask for signal switch 1
    static const uint8_t SICK_SIGNAL_SWITCH_2 = 0x40;                                   ///< Mask for signal switch 2
    static const uint8_t SICK_SIGNAL_SWITCH_3 = 0x80;                                   ///< Mask for signal switch 3

    /**
     * \struct sick_ld_config_global_tag
     * \brief A structure to aggregate the data used to configure the
     *        Sick LD global parameter values.
     */
    /**
     * \typedef sick_ld_config_global_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_ld_config_global_tag {
      uint16_t sick_sensor_id;                                                            ///< The single word sensor ID for the Sick unit
      uint16_t sick_motor_speed;                                                          ///< Nominal motor speed value: 0x0005 to 0x0014 (5 to 20)
      double sick_angle_step;                                                             ///< Difference between two laser pulse positions in 1/16th deg. (NOTE: this value must be a divisor of 5760 and be greater than 1)  
    } sick_ld_config_global_t;
    
    /**
     * \struct sick_ld_config_ethernet_tag
     * \brief A structure to aggregate the data used to configure
     *        the Sick LD unit for Ethernet.
     *
     * \todo Eventually add similar config structures for the other protocols.
     */
    /**
     * \typedef sick_ld_config_ethernet_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_ld_config_ethernet_tag {
      uint16_t sick_ip_address[4];                                                        ///< IP address in numerical form w/ leftmost part at sick_ip_address[0]
      uint16_t sick_subnet_mask[4];                                                       ///< Subnet mask for the network to which the Sick LD is assigned
      uint16_t sick_gateway_ip_address[4];                                                ///< The address of the local gateway
      uint16_t sick_node_id;                                                              ///< Single word address of the Sick LD
      uint16_t sick_transparent_tcp_port;                                                 ///< The TCP/IP transparent port associated with the Sick LD
    } sick_ld_config_ethernet_t;
    
    /**
     * \struct sick_ld_config_sector_tag
     * \brief A structure to aggregate data used to define the
     *        Sick LD's sector configuration.
     */
    /**
     * \typedef sick_ld_config_sector_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_ld_config_sector_tag {
      uint8_t sick_num_active_sectors;                                                    ///< Number of active sectors (sectors that are actually being scanned)
      uint8_t sick_num_initialized_sectors;                                               ///< Number of sectors configured w/ a function other than "not initialized" 
      uint8_t sick_active_sector_ids[SICK_MAX_NUM_SECTORS];                               ///< IDs of all active sectors
      uint8_t sick_sector_functions[SICK_MAX_NUM_SECTORS];                                ///< Function values associated w/ each of the Sick LD's sectors
      double sick_sector_start_angles[SICK_MAX_NUM_SECTORS];                              ///< Start angles for each initialized sector (deg)
      double sick_sector_stop_angles[SICK_MAX_NUM_SECTORS];                               ///< Stop angles for each sector (deg)
    } sick_ld_config_sector_t;
    
    /**
     * \struct sick_ld_identity_tag
     * \brief A structure to aggregate the fields that collectively
     *        define the identity of a Sick LD unit.
     */
    /**
     * \typedef sick_ld_identity_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_ld_identity_tag {
      std::string sick_part_number;                                                       ///< The Sick LD's part number
      std::string sick_name;                                                              ///< The name assigned to the Sick
      std::string sick_version;                                                           ///< The Sick LD's version number
      std::string sick_serial_number;                                                     ///< The Sick LD's serial number
      std::string sick_edm_serial_number;                                                 ///< The Sick LD's edm??? serial number
      std::string sick_firmware_part_number;                                              ///< The Sick LD's firmware part number 
      std::string sick_firmware_name;                                                     ///< The Sick LD's firmware name
      std::string sick_firmware_version;                                                  ///< The Sick LD's firmware version
      std::string sick_application_software_part_number;                                  ///< The Sick LD's app. software part number
      std::string sick_application_software_name;                                         ///< The Sick LD's app. software name
      std::string sick_application_software_version;                                      ///< The Sick LD's app. software version
    } sick_ld_identity_t;
    
    /**
     * \struct sick_ld_sector_data_tag
     * \brief A structure to aggregate the fields that collectively
     *        define a sector in the scan area of the Sick LD unit.
     */
    /**
     * \typedef sick_ld_sector_data_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_ld_sector_data_tag {
      unsigned int sector_num;                                                            ///< The sector number in the scan area
      unsigned int num_data_points;                                                       ///< The number of data points in the scan area
      unsigned int timestamp_start;                                                       ///< The timestamp (in ms) corresponding to the time the first measurement in the sector was taken 
      unsigned int timestamp_stop;                                                        ///< The timestamp (in ms) corresponding to the time the last measurement in the sector was taken
      unsigned int echo_values[SICK_MAX_NUM_MEASUREMENTS];                                ///< The corresponding echo/reflectivity values
      double angle_step;                                                                  ///< The angle step used for the given sector (this should be the same for all sectors)
      double angle_start;                                                                 ///< The angle at which the first measurement in the sector was acquired
      double angle_stop;                                                                  ///< The angle at which the last measurement in the sector was acquired
      double range_values[SICK_MAX_NUM_MEASUREMENTS];                                     ///< The corresponding range values (NOTE: The size of this array is intended to be large enough to accomodate various sector configs.)
      double scan_angles[SICK_MAX_NUM_MEASUREMENTS];                                      ///< The scan angles corresponding to the respective measurements
    } sick_ld_sector_data_t;
    
    /**
     * \struct sick_ld_scan_profile_tag
     * \brief A structure to aggregate the fields that collectively
     *        define the profile of a single scan acquired from the
     *        Sick LD unit.
     */
    /**
     * \typedef sick_ld_scan_profile_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_ld_scan_profile_tag {
      unsigned int profile_number;                                                        ///< The number of profiles sent to the host (i.e. the current profile number)
      unsigned int profile_counter;                                                       ///< The number of profiles gathered by the Sick LD
      unsigned int layer_num;                                                             ///< The layer number associated with a scan (this will always be 0)
      unsigned int sensor_status;                                                         ///< The status of the Sick LD sensor
      unsigned int motor_status;                                                          ///< The status of the Sick LD motor
      unsigned int num_sectors;                                                           ///< The number of sectors returned in the profile
      sick_ld_sector_data_t sector_data[SICK_MAX_NUM_SECTORS];                            ///< The sectors associated with the scan profile 
    } sick_ld_scan_profile_t;
    
    /** Primary constructor */
    SickLD( const std::string sick_ip_address = DEFAULT_SICK_IP_ADDRESS,
	    const uint16_t sick_tcp_port = DEFAULT_SICK_TCP_PORT );
    
    /** Initializes the Sick LD unit (use scan areas defined in flash) */
    void Initialize( )  throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException );

    /** Gets the sensor and motor mode of the unit */
    void GetSickStatus( unsigned int &sick_sensor_mode, unsigned int &sick_motor_mode )
      throw( SickIOException, SickTimeoutException );

    /** Sets the temporal scan configuration (until power is cycled) */
    void SetSickTempScanAreas( const double * active_sector_start_angles, const double * const active_sector_stop_angles,
			       const unsigned int num_active_sectors )
      throw( SickTimeoutException, SickIOException, SickConfigException );
    
    /** Sets the internal clock of the Sick LD unit */
    void SetSickTimeAbsolute( const uint16_t absolute_clock_time, uint16_t &new_sick_clock_time )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the internal clock of the Sick LD using the relative given time value */
    void SetSickTimeRelative( const int16_t time_delta, uint16_t &new_sick_clock_time )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Gets the internal clock time of the Sick LD unit */
    void GetSickTime( uint16_t &sick_time )
      throw( SickIOException, SickTimeoutException, SickErrorException );
  
    /** Sets the signal LEDs and switches */
    void SetSickSignals( const uint8_t sick_signal_flags = DEFAULT_SICK_SIGNAL_SET )
      throw( SickIOException, SickTimeoutException, SickErrorException );

    /** Query the Sick for its current signal settings */
    void GetSickSignals( uint8_t &sick_signal_flags ) throw( SickIOException, SickTimeoutException );
  
    /** Enables nearfield suppressive filtering (in flash) */
    void EnableNearfieldSuppression( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Disables nearfield suppressive filtering (in flash) */
    void DisableNearfieldSuppression( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Acquires measurements and related data for all active sectors */
    void GetSickMeasurements( double * const range_measurements,
			      unsigned int * const echo_measurements = NULL,
			      unsigned int * const num_measurements = NULL,
			      unsigned int * const sector_ids = NULL,
			      unsigned int * const sector_data_offsets = NULL,
			      double * const sector_step_angles = NULL,
			      double * const sector_start_angles = NULL,
			      double * const sector_stop_angles = NULL,
			      unsigned int * const sector_start_timestamps = NULL,
			      unsigned int * const sector_stop_timestamps = NULL )
      throw( SickErrorException, SickIOException, SickTimeoutException, SickConfigException );

    /** Attempts to set a new senor ID for the device (in flash) */
    void SetSickSensorID( const unsigned int sick_sensor_id )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Attempts to set a new motor speed for the device (in flash) */
    void SetSickMotorSpeed( const unsigned int sick_motor_speed )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Attempts to set a new scan resolution for the device (in flash) */
    void SetSickScanResolution( const double sick_step_angle )
      throw( SickTimeoutException, SickIOException, SickConfigException );

    /** Attempts to set the global params and the active scan sectors for the device (in flash) */
    void SetSickGlobalParamsAndScanAreas( const unsigned int sick_motor_speed,
					  const double sick_step_angle,
					  const double * const active_sector_start_angles,
					  const double * const active_sector_stop_angles,
					  const unsigned int num_active_sectors )
      throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException );

    /** Attempts to set the active scan sectors for the device (in flash) */
    void SetSickScanAreas( const double * const active_sector_start_angles,
			   const double * const active_sector_stop_angles,
			   const unsigned int num_active_sectors )
      throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException );

    /** Resets the Sick LD using the given reset level */
    void ResetSick( const unsigned int reset_level = SICK_WORK_SERV_RESET_INIT_CPU )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Returns the number of active/measuring sectors */
    unsigned int GetSickNumActiveSectors( ) const;
  
    /** Acquire the Sick LD's sensor ID */
    unsigned int GetSickSensorID( ) const;

    /** Acquire the Sick LD's current motor speed in Hz */
    unsigned int GetSickMotorSpeed( ) const;

    /** Acquire the Sick LD's current scan resolution */
    double GetSickScanResolution( ) const;
  
    /** Acquire the current IP address of the Sick */
    std::string GetSickIPAddress( ) const;
  
    /** Acquire the subnet mask for the Sick */
    std::string GetSickSubnetMask( ) const;
  
    /** Acquire the IP address of the Sick gateway */
    std::string GetSickGatewayIPAddress( ) const;
  
    /** Acquire the Sick LD's part number */
    std::string GetSickPartNumber( ) const;

    /** Acquire the Sick LD's name */
    std::string GetSickName( ) const;

    /** Acquire the Sick LD's version number */
    std::string GetSickVersion( ) const;

    /** Acquire the Sick LD's serial number */
    std::string GetSickSerialNumber( ) const;

    /** Acquire the Sick LD's EDM serial number */
    std::string GetSickEDMSerialNumber( ) const;

    /** Acquire the Sick LD's firmware part number */
    std::string GetSickFirmwarePartNumber( ) const;

    /** Acquire the Sick LD's firmware number */
    std::string GetSickFirmwareName( ) const;

    /** Acquire the Sick LD's firmware version */
    std::string GetSickFirmwareVersion( ) const;

    /** Acquire the Sick LD's application software part number */
    std::string GetSickAppSoftwarePartNumber( ) const;

    /** Acquire the Sick LD's application software name */
    std::string GetSickAppSoftwareName( ) const;

    /** Acquire the Sick LD's application software version number */
    std::string GetSickAppSoftwareVersionNumber( ) const;

    /** Acquire the Sick LD's status as a printable string */
    std::string GetSickStatusAsString() const;

    /** Acquire the Sick LD's identity as a printable string */
    std::string GetSickIdentityAsString() const;

    /** Acquire the Sick LD's global config as a printable string */
    std::string GetSickGlobalConfigAsString() const;

    /** Acquire the Sick LD's Ethernet config as a printable string */
    std::string GetSickEthernetConfigAsString() const;

    /** Acquire the Sick LD's sector config as a printable string */
    std::string GetSickSectorConfigAsString() const;
    
    /** Acquire the total scan area (in degrees) being scanned by the Sick LD */
    double GetSickScanArea( ) const;
  
    /** Prints the Sick LD's status information */
    void PrintSickStatus( ) const;

    /** Prints the Sick LD's identity information */
    void PrintSickIdentity( ) const;

    /** Prints the global configuration parameter values */
    void PrintSickGlobalConfig( ) const;

    /** Prints the Ethernet configuration parameter values */
    void PrintSickEthernetConfig( ) const;

    /** Prints the Sick Sector configuration */
    void PrintSickSectorConfig( ) const;
  
    /** Uninitializes the Sick LD unit */
    void Uninitialize( ) throw( SickIOException, SickTimeoutException, SickErrorException, SickThreadException );

    /** Destructor */
    ~SickLD();

  private:

    /** The Sick LD IP address */
    std::string _sick_ip_address;

    /** The Sick LD TCP port number */
    uint16_t _sick_tcp_port;

    /** Sick LD socket structure */
    unsigned int _socket;
  
    /** Sick LD socket address structure */
    struct sockaddr_in _sick_inet_address_info;

    /** The current sensor mode */
    uint8_t _sick_sensor_mode;

    /** The mode of the motor */
    uint8_t _sick_motor_mode;

    /** Indicates whether the Sick LD is currently streaming range data */
    bool _sick_streaming_range_data;

    /** Indicates whether the Sick LD is currently streaming range and echo data */
    bool _sick_streaming_range_and_echo_data;
  
    /** The identity structure for the Sick */
    sick_ld_identity_t _sick_identity;

    /** The current global configuration for the unit */
    sick_ld_config_global_t _sick_global_config;
  
    /** The current Ethernet configuration for the unit */
    sick_ld_config_ethernet_t _sick_ethernet_config;

    /** The current sector configuration for the unit */
    sick_ld_config_sector_t _sick_sector_config;

    /** Setup the connection parameters and establish TCP connection! */
    void _setupConnection( ) throw( SickIOException, SickTimeoutException );
  
    /** Synchronizes the driver state with the Sick LD (used for initialization) */
    void _syncDriverWithSick( ) throw( SickIOException, SickTimeoutException, SickErrorException );

    /** Set the function for a particular scan secto */
    void _setSickSectorFunction( const uint8_t sector_number, const uint8_t sector_function,
				 const double sector_angle_stop, const bool write_to_flash = false )
       throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );
  
    /** Acquires the given Sector's function (i.e. current config) */
    void _getSickSectorFunction( const uint8_t sector_num, uint8_t &sector_function, double &sector_stop_angle )
      throw( SickErrorException, SickTimeoutException, SickIOException );
  
    /** Sets the Sick LD to IDLE mode */
    void _setSickSensorModeToIdle( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Sets the Sick LD to ROTATE mode */
    void _setSickSensorModeToRotate( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Sets the Sick LD to MEASURE mode */
    void _setSickSensorModeToMeasure( ) throw( SickErrorException, SickTimeoutException, SickIOException );
  
    /** Sets the Sick LD's sensor mode to IDLE (laser off, motor off) */
    void _setSickSensorMode( const uint8_t new_sick_sensor_mode )
      throw( SickErrorException, SickTimeoutException, SickIOException );
  
    /** Requests n range measurement profiles from the Sick LD */
    void _getSickScanProfiles( const uint16_t profile_format, const uint16_t num_profiles = DEFAULT_SICK_NUM_SCAN_PROFILES )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Parses a sequence of bytes and populates the profile_data struct w/ the results */
    void _parseScanProfile( uint8_t * const src_buffer, sick_ld_scan_profile_t &profile_data ) const;

    /** Cancels the active data stream */
    void _cancelSickScanProfiles( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Turns nearfield suppression on/off */
    void _setSickFilter( const uint8_t suppress_code )
      throw( SickErrorException, SickTimeoutException, SickIOException );
  
    /** Stores an image of the Sick LD's identity locally */
    void _getSickIdentity( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its sensor and motor status */
    void _getSickStatus( ) throw( SickTimeoutException, SickIOException );

    /** Sets the Sick LD's global configuration (in flash) */
    void _setSickGlobalConfig( const uint8_t sick_sensor_id, const uint8_t sick_motor_speed, const double sick_angle_step )
      throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Query the Sick for its global configuration parameters */
    void _getSickGlobalConfig( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Query the Sick for its Ethernet configuration parameters */
    void _getSickEthernetConfig( ) throw( SickErrorException, SickTimeoutException, SickIOException );

    /** Acquires the configuration (function and stop angle) for each sector */
    void _getSickSectorConfig( ) throw( SickErrorException, SickTimeoutException, SickIOException );
  
    /** Query the Sick for ID information */
    void _getIdentificationString( const uint8_t id_request_code, std::string &id_return_string )
      throw( SickTimeoutException, SickIOException );
  
    /** Query the Sick for its sensor part number */
    void _getSensorPartNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its assigned name */
    void _getSensorName( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its version number */
    void _getSensorVersion( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its serial number */
    void _getSensorSerialNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for its EDM unit's serial number */
    void _getSensorEDMSerialNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the part number of its firmware */
    void _getFirmwarePartNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the name of its firmware */
    void _getFirmwareName( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the version of the firmware */
    void _getFirmwareVersion( ) throw( SickTimeoutException, SickIOException );

    /** Query the part number of the application software */
    void _getApplicationSoftwarePartNumber( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the application name */
    void _getApplicationSoftwareName( ) throw( SickTimeoutException, SickIOException );

    /** Query the Sick for the application software version */
    void _getApplicationSoftwareVersion( ) throw( SickTimeoutException, SickIOException );

    /** Allows setting the global parameters and scan area definition (in flash) */
    void _setSickGlobalParamsAndScanAreas( const unsigned int sick_motor_speed, const double sick_step_angle,
					   const double * const active_sector_start_angles,
					   const double * const active_sector_stop_angles,
					   const unsigned int num_active_sectors )
       throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException );

    /** Allows setting a temporary (until a device reset) sector configuration on the device */
    void _setSickTemporaryScanAreas( const double * const active_sector_start_angles,
				     const double * const active_sector_stop_angles,
				     const unsigned int num_active_sectors )
      throw( SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the sick sector configuration */
    void _setSickSectorConfig( const unsigned int * const sector_functions, const double * const sector_stop_angles,
			       const unsigned int num_sectors, const bool write_to_flash = false )
      throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException );

    /** Sets the signals for the device */
    void _setSickSignals( const uint8_t sick_signal_flags = DEFAULT_SICK_SIGNAL_SET )
      throw( SickIOException, SickTimeoutException, SickErrorException );
  
    /** Send a message, get the reply from the Sick LD and check it */
    void _sendMessageAndGetReply( const SickLDMessage &send_message, SickLDMessage &recv_message,
				  const unsigned int timeout_value = DEFAULT_SICK_MESSAGE_TIMEOUT ) 
      throw( SickIOException, SickTimeoutException );

    /** Flushed the TCP receive buffer */
    void _flushTCPRecvBuffer( ) throw ( SickIOException, SickThreadException );
    
    /** Teardown the connection to the Sick LD */
    void _teardownConnection( ) throw( SickIOException );

    /** Generates a device-ready sector set given only an active sector spec. */
    void _generateSickSectorConfig( const double * const active_sector_start_angles,
				    const double * const active_sector_stop_angles,
				    const unsigned int num_active_sectors,
				    const double sick_step_angle,
				    unsigned int * const sector_functions, 
				    double * const sector_stop_angles,
				    unsigned int &num_sectors ) const;
  
    /** Converts odometry ticks to an equivalent angle */
    double _ticksToAngle( const uint16_t ticks ) const;

    /** Converts angle to an equivalent representation in odometer ticks */
    uint16_t _angleToTicks( const double angle ) const;

    /** Computes the mean pulse frequency for the given config */
    double _computeMeanPulseFrequency( const double active_scan_area, const double curr_motor_speed,
				       const double curr_angular_resolution ) const;
  
    /** Computes the total pulse frequency for the given config */
    double _computeMaxPulseFrequency( const double total_scan_area, const double curr_motor_speed,
				      const double curr_angular_resolution ) const;

    /** Indicates whether a given sensor ID is valid for the device */
    bool _validSickSensorID( const unsigned int sick_sensor_id ) const;

    /** Indicates whether a given motor speed is valid for the device */
    bool _validSickMotorSpeed( const unsigned int sick_motor_speed ) const;

    /** Indicates whether a given motor speed is valid for the device */
    bool _validSickScanResolution( const double sick_step_angle, const double * const active_sector_start_angles,
				   const double * const active_sector_stop_angles, const unsigned int num_active_sectors ) const;

    /** Indicates whether the given configuration yields a valid max and mean pulse frequency */
    bool _validPulseFrequency( const unsigned int sick_motor_speed, const double sick_step_angle ) const;
  
    /** Indicates whether the given configuration yields a valid max and mean pulse frequency */
    bool _validPulseFrequency( const unsigned int sick_motor_speed, const double sick_step_angle,
			       const double * const active_sector_start_angles,
			       const double * const active_sector_stop_angles,
			       const unsigned int num_active_sectors ) const;
  
    /** Returns the scanning area for the device given the current sector configuration */
    double _computeScanArea( const double sick_step_angle, const double * const sector_start_angles,
			     const double * const sector_stop_angles, const unsigned int num_sectors ) const;

    /** Reorders given sector angle sets */
    void _sortScanAreas( double * const sector_start_angles, double * const sector_stop_angles,
			 const unsigned int num_sectors ) const;

    /** Checks the given sector arguments for overlapping regions yielding an invalid configuration */
    bool _validActiveSectors( const double * const sector_start_angles, const double * const sector_stop_angles,
			      const unsigned int num_active_sectors ) const;
    
    /** Indicates whether the supplied profile format is currently supported by the driver */
    bool _supportedScanProfileFormat( const uint16_t profile_format ) const; 

    /** Prints data corresponding to a single scan sector (data obtained using GET_PROFILE) */
    void _printSectorProfileData( const sick_ld_sector_data_t &sector_data ) const;

    /** Prints the data corresponding to the given scan profile (for debugging purposes) */
    void _printSickScanProfile( const sick_ld_scan_profile_t profile_data, const bool print_sector_data = true ) const;

    /** Returns the corresponding work service subcode required to transition the Sick LD to the given sensor mode. */
    uint8_t _sickSensorModeToWorkServiceSubcode( const uint8_t sick_sensor_mode ) const;

    /** Converts _sick_sensor_mode to a representative string */
    std::string _sickSensorModeToString( const uint8_t sick_sensor_mode ) const;
 
    /** Converts _sick_motor_mode to a representative string */
    std::string _sickMotorModeToString( const uint8_t sick_motor_mode ) const;

    /** Converts the specified trans measurement mode return value to a string */
    std::string _sickTransMeasureReturnToString( const uint8_t return_value ) const;
  
    /** Converts the specified reset level to a representative string */
    std::string _sickResetLevelToString( const uint16_t reset_level ) const;
  
    /** Converts Sick LD sector configuration word to a representative string */
    std::string _sickSectorFunctionToString( const uint16_t sick_sector_function ) const;
  
    /** Converts a given scan profile format to a string for friendlier output */
    std::string _sickProfileFormatToString( const uint16_t profile_format ) const;

    /** Prints the initialization footer */
    void _printInitFooter( ) const;

  };

} //namespace SickToolbox
  
#endif /* SICK_LD_HH */
