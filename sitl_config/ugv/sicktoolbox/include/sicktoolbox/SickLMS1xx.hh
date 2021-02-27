/*!
 * \file SickLMS1xx.hh
 * \brief Defines the SickLMS1xx class for working with the
 *        Sick LMS1xx laser range finders.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_LMS_1XX_HH
#define SICK_LMS_1XX_HH

/* Macros */
#define DEFAULT_SICK_LMS_1XX_IP_ADDRESS                   "192.168.0.1"                 ///< Default IP Address
#define DEFAULT_SICK_LMS_1XX_TCP_PORT                            (2111)                 ///< Sick LMS 1xx TCP/IP Port
#define DEFAULT_SICK_LMS_1XX_CONNECT_TIMEOUT                  (1000000)                 ///< Max time for establishing connection (usecs)
#define DEFAULT_SICK_LMS_1XX_MESSAGE_TIMEOUT                  (5000000)                 ///< Max time for reply (usecs)
#define DEFAULT_SICK_LMS_1XX_STATUS_TIMEOUT                  (60000000)                 ///< Max time it should take to change status  

#define SICK_LMS_1XX_SCAN_AREA_MIN_ANGLE                      (-450000)                 ///< -45 degrees (1/10000) degree
#define SICK_LMS_1XX_SCAN_AREA_MAX_ANGLE                      (2250000)                 ///< 225 degrees (1/10000) degree

/* Definition dependencies */
#include <string>
#include <arpa/inet.h>

#include "SickLIDAR.hh"
#include "SickLMS1xxBufferMonitor.hh"
#include "SickLMS1xxMessage.hh"
#include "SickException.hh"

/**
 * \namespace SickToolbox
 * \brief Encapsulates the Sick LIDAR Matlab/C++ toolbox
 */
namespace SickToolbox {

  /**
   * \class SickLMS1xx
   * \brief Provides a simple driver interface for working with the
   *        Sick LD-OEM/Sick LD-LRS long-range models via Ethernet.
   */
  class SickLMS1xx : public SickLIDAR< SickLMS1xxBufferMonitor, SickLMS1xxMessage > {

  public:
    
    static const int SICK_LMS_1XX_MAX_NUM_MEASUREMENTS = 1082;                         ///< LMS 1xx max number of measurements

    /*!
     * \enum sick_lms_1xx_status_t 
     * \brief Defines the Sick LMS 1xx status.
     * This enum lists all of the Sick LMS 1xx status.
     */
    enum sick_lms_1xx_status_t {
      
      SICK_LMS_1XX_STATUS_UNKNOWN = 0x00,                                              ///< LMS 1xx status undefined
      SICK_LMS_1XX_STATUS_INITIALIZATION = 0x01,                                       ///< LMS 1xx initializing
      SICK_LMS_1XX_STATUS_CONFIGURATION = 0x02,                                        ///< LMS 1xx configuration
      SICK_LMS_1XX_STATUS_IDLE = 0x03,                                                 ///< LMS 1xx is idle
      SICK_LMS_1XX_STATUS_ROTATED = 0x04,                                              ///< LMS 1xx mirror rotating
      SICK_LMS_1XX_STATUS_IN_PREP = 0x05,                                              ///< LMS 1xx in preparation
      SICK_LMS_1XX_STATUS_READY = 0x06,                                                ///< LMS 1xx is ready
      SICK_LMS_1XX_STATUS_READY_FOR_MEASUREMENT = 0x07                                 ///< LMS 1xx is ready to give measurements 
      
    };        
    
    /*!
     * \enum sick_lms_1xx_scan_format_t 
     * \brief Defines the Sick LMS 1xx scan types
     * This enum is for specifiying the desired scan returns.
     */
    enum sick_lms_1xx_scan_format_t {
      
      SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_NONE = 0x00,                   ///< Single-pulse dist, no reflect
      SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_8BIT = 0x01,                   ///< Single-pulse dist, 8bit reflect
      SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_16BIT = 0x02,                  ///< Single-pulse dist, 16bit reflect            
      SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_NONE = 0x03,                   ///< Double-pulse dist, no reflect
      SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_8BIT = 0x04,                   ///< Double-pulse dist, 8bit reflect
      SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_16BIT = 0x05,                  ///< Double-pulse dist, 16bit reflect          
      SICK_LMS_1XX_SCAN_FORMAT_UNKNOWN = 0xFF                                           ///< Unknown format
      
    };

    /*!
     * \enum sick_lms_1xx_scan_freq_t 
     * \brief Defines the Sick LMS 1xx scan frequency.
     * This enum lists all of the Sick LMS 1xx scan frequencies.
     */
    enum sick_lms_1xx_scan_freq_t {

      SICK_LMS_1XX_SCAN_FREQ_UNKNOWN = 0x00,                                            ///< LMS 1xx scan freq unknown
      SICK_LMS_1XX_SCAN_FREQ_25 = 0x09C4,                                               ///< LMS 1xx scan freq 25Hz
      SICK_LMS_1XX_SCAN_FREQ_50 = 0X1388                                                ///< LMS 1xx scan freq 50Hz

    };

    /*!
     * \enum sick_lms_1xx_scan_res_t 
     * \brief Defines the Sick LMS 1xx scan resolution.
     * This enum lists all of the Sick LMS 1xx scan resolutions.
     */
    enum sick_lms_1xx_scan_res_t {

      SICK_LMS_1XX_SCAN_RES_UNKNOWN = 0x00,                                             ///< LMS 1xx scab res unknown
      SICK_LMS_1XX_SCAN_RES_25 = 0x09C4,                                                ///< LMS 1xx scan res 0.25 deg
      SICK_LMS_1XX_SCAN_RES_50 = 0x1388                                                 ///< LMS 1xx scan res 0.50 deg

    };

    /** Primary constructor */
    SickLMS1xx( const std::string sick_ip_address = DEFAULT_SICK_LMS_1XX_IP_ADDRESS,
		const uint16_t sick_tcp_port = DEFAULT_SICK_LMS_1XX_TCP_PORT );
    
    /** Initializes the Sick LD unit (use scan areas defined in flash) */
    void Initialize( const bool disp_banner = true ) throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException );

    /** Sets the Sick LMS 1xx scan frequency and scan resolution */
    void SetSickScanFreqAndRes( const sick_lms_1xx_scan_freq_t scan_freq,
				const sick_lms_1xx_scan_res_t scan_res ) throw( SickTimeoutException, SickIOException, SickErrorException ); 

    /** Get the Sick LMS 1xx scan frequency */
    sick_lms_1xx_scan_freq_t GetSickScanFreq( ) const throw ( SickIOException );
    
    /** Get the Sick LMS 1xx scan resolution */
    sick_lms_1xx_scan_res_t GetSickScanRes( ) const throw ( SickIOException );

    /** Get the Sick LMS 1xx scan start angle */
    double GetSickStartAngle( ) const throw ( SickIOException );

    /** Get the Sick LMS 1xx scan stop angle */
    double GetSickStopAngle( ) const throw ( SickIOException );    

    /** Sets the sick scan data format */
    void SetSickScanDataFormat( const sick_lms_1xx_scan_format_t scan_format ) throw( SickTimeoutException, SickIOException, SickThreadException, SickErrorException );
    
    /** Get the Sick Range Measurements */
    void GetSickMeasurements( unsigned int * const range_1_vals,
			      unsigned int * const range_2_vals,
			      unsigned int * const reflect_1_vals,
			      unsigned int * const reflect_2_vals,
			      unsigned int & num_measurements,
			      unsigned int * const dev_status = NULL ) throw ( SickIOException, SickConfigException, SickTimeoutException );

    /** Uninitializes the Sick LD unit */
    void Uninitialize( const bool disp_banner = true ) throw( SickIOException, SickTimeoutException, SickErrorException, SickThreadException );

    /** Utility function for converting integer to scan frequency */
    sick_lms_1xx_scan_freq_t IntToSickScanFreq( const int scan_freq ) const;

    /** Utility function for converting scan frequency to integer */
    int SickScanFreqToInt( const sick_lms_1xx_scan_freq_t scan_freq ) const;
    
    /** Utility function for converting double to scan resolution */
    sick_lms_1xx_scan_res_t DoubleToSickScanRes( const double scan_res ) const;

    /** Utility function for converting scan resolution to double */
    double SickScanResToDouble( const sick_lms_1xx_scan_res_t scan_res ) const;
    
    /** Destructor */
    ~SickLMS1xx();

  private:

    /*!
     * \struct sick_lms_1xx_scan_config_tag
     * \brief A structure for aggregrating the
     *        Sick LMS 1xx configuration params.
     */
    /*!
     * \typedef sick_lms_1xx_scan_config_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_1xx_scan_config_tag {
      sick_lms_1xx_scan_freq_t sick_scan_freq;                                          ///< Sick scan frequency
      sick_lms_1xx_scan_res_t sick_scan_res;                                            ///< Sick scan resolution      
      int32_t sick_start_angle;                                                         ///< Sick scan area start angle
      int32_t sick_stop_angle;                                                          ///< Sick scan area stop angle
    } sick_lms_1xx_scan_config_t;
    
    /** The Sick LMS 1xx IP address */
    std::string _sick_ip_address;

    /** The Sick LMS 1xx TCP port number */
    uint16_t _sick_tcp_port;

    /** Sick LMS 1xx socket address struct */
    struct sockaddr_in _sick_inet_address_info;

    /** Sick LMS 1xx configuration struct */
    sick_lms_1xx_scan_config_t _sick_scan_config;

    /** Sick LMS 1xx current scan data format */
    sick_lms_1xx_scan_format_t _sick_scan_format;    
    
    /** Sick LMS 1xx configuration struct */
    sick_lms_1xx_status_t _sick_device_status;

    /** Sick LMS 1xx temperature status */
    bool _sick_temp_safe;    
    
    /** Sick LMS 1xx streaming status */
    bool _sick_streaming;
    
    /** Setup the connection parameters and establish TCP connection! */
    void _setupConnection( ) throw( SickIOException, SickTimeoutException );

    /** Re-initialized the device */
    void _reinitialize( ) throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException );     

    /** Teardown the connection to the Sick LD */
    void _teardownConnection( ) throw( SickIOException );

    /** Acquire the latest Sick LMS's status */
    void _updateSickStatus( ) throw( SickTimeoutException, SickIOException );

    /** Acquire the Sick LMS's scan config */
    void _getSickScanConfig( ) throw( SickTimeoutException, SickIOException );

    /** Sets the scan configuration (volatile, does not write to EEPROM) */
    void _setSickScanConfig( const sick_lms_1xx_scan_freq_t scan_freq,
			     const sick_lms_1xx_scan_res_t scan_res,
			     const int start_angle, const int stop_angle ) throw( SickTimeoutException, SickIOException, SickErrorException );
    
    /** Set access mode for configuring device */
    void _setAuthorizedClientAccessMode( ) throw( SickTimeoutException, SickErrorException, SickIOException );

    /** Save configuration parameters to EEPROM */
    void _writeToEEPROM( ) throw( SickTimeoutException, SickIOException );

    /** Send the message w/o waiting for a reply */
    void _sendMessage( const SickLMS1xxMessage &send_message ) const throw ( SickIOException );
    
    /** Send the message and grab expected reply */
    void _sendMessageAndGetReply( const SickLMS1xxMessage &send_message,
				  SickLMS1xxMessage &recv_message,
				  const std::string reply_command_code,
				  const std::string reply_command,
				  const unsigned int timeout_value = DEFAULT_SICK_LMS_1XX_MESSAGE_TIMEOUT,
				  const unsigned int num_tries = 1 ) throw( SickIOException, SickTimeoutException );

    /** Receive a message */
    void _recvMessage( SickLMS1xxMessage &sick_message ) const throw ( SickTimeoutException );
    
    /** Start device measuring */
    void _startMeasuring( ) throw ( SickTimeoutException, SickIOException );

    /** Stop device measuring */
    void _stopMeasuring( ) throw ( SickTimeoutException, SickIOException );

    /** Request a data data stream type */
    void _requestDataStream( ) throw ( SickTimeoutException, SickConfigException, SickIOException );
    
    /** Start streaming measurements */
    void _startStreamingMeasurements(  )throw( SickTimeoutException, SickIOException );

    /** Stop streaming measurements */
    void _stopStreamingMeasurements( const bool disp_banner = true ) throw( SickTimeoutException, SickIOException );

    /** Set device to measuring mode */
    void _checkForMeasuringStatus( unsigned int timeout_value = DEFAULT_SICK_LMS_1XX_STATUS_TIMEOUT ) throw( SickTimeoutException, SickIOException );

    /** Sets the sick scan data format */
    void _setSickScanDataFormat( const sick_lms_1xx_scan_format_t scan_format ) throw( SickTimeoutException, SickIOException, SickThreadException, SickErrorException );
    
    /** Restore device to measuring mode */
    void _restoreMeasuringMode( ) throw( SickTimeoutException, SickIOException );

    /** Ensures a feasible scan area */
    bool _validScanArea( const int start_angle, const int stop_angle ) const;
    
    /** Utility function to convert int to status */
    sick_lms_1xx_status_t _intToSickStatus( const int status ) const;

    /** Utility function for printing Sick scan config */
    void _printSickScanConfig( ) const;
    
    /** Utility function for printing footer after initialization */
    void _printInitFooter( ) const;

    /** Utility function for returning scan format as string */
    std::string _sickScanDataFormatToString( const sick_lms_1xx_scan_format_t scan_format ) const;

    /** Utility function for converting sick freq to doubles */
    double _convertSickAngleUnitsToDegs( const int sick_angle ) const { return ((double)sick_angle)/10000; }

    /** Utility function for converting sick Hz values ints */
    unsigned int  _convertSickFreqUnitsToHz( const unsigned int sick_freq ) const { return (unsigned int)(((double)sick_freq)/100); }    
    
    /** Utility function to convert config error int to str */
    std::string _intToSickConfigErrorStr( const int error ) const;

    /** Utility function to locate substring in string */
    bool _findSubString( const char * const str, const char * const substr, const unsigned int str_length, const unsigned int substr_length,
			 unsigned int &substr_pos, unsigned int start_pos = 0 ) const;

    /** Utility function for extracting next integer from tokenized string */
    char * _convertNextTokenToUInt( char * const str_buffer, unsigned int & num_val, const char * const delimeter = " " ) const;
    
  };

  /*!
   * \typedef sick_lms_1xx_status_t
   * \brief Makes working w/ SickLMS1xx::sick_lms_1xx_status_t a bit easier
   */
  typedef SickLMS1xx::sick_lms_1xx_status_t sick_lms_1xx_status_t;

  /*!
   * \typedef sick_lms_1xx_scan_format_t
   * \brief Makes working w/ SickLMS1xx::sick_lms_1xx_scan_format_t a bit easier
   */
  typedef SickLMS1xx::sick_lms_1xx_scan_format_t sick_lms_1xx_scan_format_t;

  /*!
   * \typedef sick_lms_1xx_scan_freq_t
   * \brief Makes working w/ SickLMS1xx::sick_lms_1xx_scan_freq_t a bit easier
   */
  typedef SickLMS1xx::sick_lms_1xx_scan_freq_t sick_lms_1xx_scan_freq_t;

  /*!
   * \typedef sick_lms_1xx_scan_res_t
   * \brief Makes working w/ SickLMS1xx::sick_lms_1xx_scan_res_t a bit easier
   */
  typedef SickLMS1xx::sick_lms_1xx_scan_res_t sick_lms_1xx_scan_res_t;  

  
} //namespace SickToolbox
  
#endif /* SICK_LMS_1XX_HH */
