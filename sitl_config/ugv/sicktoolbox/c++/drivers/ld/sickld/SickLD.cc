/*!
 * \file SickLD.cc
 * \brief Implements the SickLD driver class.
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

/* Auto-generated header */
#include <sicktoolbox/SickConfig.hh>

/* Implementation dependencies */
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>            // for converting numerical values to strings
#include <sys/socket.h>       // for socket function definitions
#include <arpa/inet.h>        // for sockaddr_in, inet_addr, and htons
#include <sys/ioctl.h>        // for using ioctl functionality for the socket input buffer
#include <unistd.h>           // for select functionality (e.g. FD_SET, etc...)
#include <sys/types.h>        // for fd data types
#include <sys/time.h>         // for select timeout parameter
#include <fcntl.h>            // for getting file flags
#include <pthread.h>          // for POSIX threads
#include <sstream>            // for parsing ip addresses
#include <vector>             // for returning the results of parsed strings
#include <errno.h>            // for timing connect()

#include <sicktoolbox/SickLD.hh>
#include <sicktoolbox/SickLDMessage.hh>
#include <sicktoolbox/SickLDBufferMonitor.hh>
#include <sicktoolbox/SickLDUtility.hh>   
#include <sicktoolbox/SickException.hh>

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   * \param sick_ip_address The ip address of the Sick LD
   * \param sick_tcp_port The TCP port associated w/ the Sick LD server
   */
  SickLD::SickLD( const std::string sick_ip_address, const uint16_t sick_tcp_port ) :
    SickLIDAR< SickLDBufferMonitor, SickLDMessage >( ),
    _sick_ip_address(sick_ip_address),
    _sick_tcp_port(sick_tcp_port),
    _sick_sensor_mode(SICK_SENSOR_MODE_UNKNOWN),
    _sick_motor_mode(SICK_MOTOR_MODE_UNKNOWN),
    _sick_streaming_range_data(false),
    _sick_streaming_range_and_echo_data(false)
  {
    /* Initialize the sick identity */
    _sick_identity.sick_part_number =
      _sick_identity.sick_name =
      _sick_identity.sick_version = 
      _sick_identity.sick_serial_number =
      _sick_identity.sick_edm_serial_number =
      _sick_identity.sick_firmware_part_number =
      _sick_identity.sick_firmware_name =
      _sick_identity.sick_firmware_version =
      _sick_identity.sick_application_software_part_number =
      _sick_identity.sick_application_software_name =
      _sick_identity.sick_application_software_version = "UNKNOWN";

    /* Initialize the global configuration structure */
    memset(&_sick_global_config,0,sizeof(sick_ld_config_global_t));

    /* Initialize the Ethernet configuration structure */
    memset(&_sick_ethernet_config,0,sizeof(sick_ld_config_ethernet_t));

    /* Initialize the sector configuration structure */
    memset(&_sick_sector_config,0,sizeof(sick_ld_config_sector_t));
  }

  /**
   * A standard destructor
   */
  SickLD::~SickLD( ) { }

  /**
   * \brief Initializes the driver and syncs it with Sick LD unit. Uses sector config given in flash.
   */
  void SickLD::Initialize( ) throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException ) {

    std::cout << "\t*** Attempting to initialize the Sick LD..." << std::endl; 

    try {
      
      /* Attempt to connect to the Sick LD */
      std::cout << "\tAttempting to connect to Sick LD @ " << _sick_ip_address << ":" << _sick_tcp_port << std::endl;
      _setupConnection();
      std::cout << "\t\tConnected to Sick LD!" << std::endl;

      /* Start the buffer monitor */
      std::cout << "\tAttempting to start buffer monitor..." << std::endl;
      _startListening();
      std::cout << "\t\tBuffer monitor started!" << std::endl;
    
      /* Ok, lets sync the driver with the Sick */
      std::cout << "\tAttempting to sync driver with Sick LD..." << std::endl;
      _syncDriverWithSick();
      
    }
    
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickLD::Initialize - Unknown exception!" << std::endl;
      throw;
    }
    
    std::cout << "\t\tSynchronized!" << std::endl;

    _sick_initialized = true;
    _printInitFooter();

    /* Success */
  }

  /**
   * \brief Acquires the status of the Sick from the device
   * \param &sick_sensor_mode The returned sensor mode of the device
   * \param &sick_motor_mode The returned motor mode of the device
   */
  void SickLD::GetSickStatus( unsigned int &sick_sensor_mode, unsigned int &sick_motor_mode )
    throw( SickIOException, SickTimeoutException ){

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::GetSickStatus: Device NOT Initialized!!!");
    }
  
    /* Acquire the sensor and motor mode from the device */
    try {
      _getSickStatus();
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickLD::GetSickStatus - Unknown exception!" << std::endl;
      throw;
    }
    
    /* Now that the driver is updated, assign the return values */
    sick_sensor_mode = _sick_sensor_mode;
    sick_motor_mode = _sick_motor_mode;
  
    /* Success */
  }

  /**
   * \brief Set the temporary scan areas for the Sick LD
   * \param *active_sector_start_angles Angles marking the beginning of each desired active sector/area
   * \param *active_sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_active_sectors The number of active sectors
   *
   * NOTE: The active scan areas set at this point are only maintained until a device
   *       reset occurs. In other words, they are not written to flash.
   */ 
  void SickLD::SetSickTempScanAreas( const double * active_sector_start_angles, const double * active_sector_stop_angles,
				     const unsigned int num_active_sectors )
        throw( SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLD::SetSickTempScanAreas: Device NOT Initialized!!!");
    }
    
    /* Do the standard initialization */
    try {
      
      /* Set the temporary scan configuration */
      std::cout << "\tAttempting to set desired scan config..." << std::endl;
      _setSickTemporaryScanAreas(active_sector_start_angles,active_sector_stop_angles,num_active_sectors);
      std::cout << "\t\tUsing desired scan area(s)!" << std::endl;

    }
    
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    catch(...) {
      std::cerr << "SickLD::Initialize - Unknown exception!" << std::endl;
      throw;
    }
    
    /* Success */    
  }
  
  
  /**
   * \brief Set the absolute time of the Sick LD unit.
   * \param absolute_clock_time The absolute clock time in milliseconds
   * \param new_sick_clock_time The clock time in milliseconds returned from the device
   * 
   * ALERT: If the Sick LD is in MEASUREMENT mode, this method will first set the device
   *        to ROTATE mode after killing any active data streams (as per the protocol).
   */
  void SickLD::SetSickTimeAbsolute( const uint16_t absolute_clock_time, uint16_t &new_sick_clock_time ) 
    throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLD::SetSickTimeAbsolute: Device NOT Initialized!!!");
    }
  
    /* Ensure the device is not in MEASURE mode */
    if (_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) {

      /* If it is then set it to rotate */
      try {
	_setSickSensorModeToRotate();
      }
               
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::SetSickTimeAbsolute: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }
  
    std::cout << "\tSetting Sick LD absolute clock time..." << std::endl;
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service codes */
    payload_buffer[0] = SICK_CONF_SERV_CODE;
    payload_buffer[1] = SICK_CONF_SERV_SET_TIME_ABSOLUTE;

    /* Set the new time value */
    uint16_t temp_buffer = host_to_sick_ld_byte_order(absolute_clock_time);
    memcpy(&payload_buffer[2],&temp_buffer,2);

    /* Create the Sick LD send/receive message objects */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;  
  
    /* Send the message and check for the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
            
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,4);

    /* Get the message payload */
    recv_message.GetPayload(payload_buffer);
  
    /* Extract the new Sick LD clock time from the response */
    uint16_t clock_time;
    memcpy(&clock_time,&payload_buffer[2],2);
    new_sick_clock_time = sick_ld_to_host_byte_order(clock_time);

    std::cout << "\t\tClock time set!" << std::endl;
  
    /* Success */
  }

  /**
   * \brief Set the relative time of the Sick LD unit.
   * \param delta_time The relative clock time in milliseconds.
   * \param &new_sick_clock_time The new time as reported by the Sick LD.
   *
   * ALERT: If the Sick LD is in MEASUREMENT mode, this method will first set the device
   *        to ROTATE mode after killing any active data streams (as per the protocol).
   */
  void SickLD::SetSickTimeRelative( const int16_t delta_time, uint16_t &new_sick_clock_time ) 
    throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLD::SetSickTimeRelative: Device NOT Initialized!!!");
    }
  
    /* Ensure the device is not in MEASURE mode */
    if (_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) {
      
      /* If it is then set it to rotate */
      try {
	_setSickSensorModeToRotate();
      }
      
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::SetSickTimeRelative: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }
    
    std::cout << "\tSetting Sick LD relative clock time..." << std::endl;
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service codes */
    payload_buffer[0] = SICK_CONF_SERV_CODE;
    payload_buffer[1] = SICK_CONF_SERV_SET_TIME_RELATIVE;

    /* Set the new time value */
    uint16_t temp_buffer = host_to_sick_ld_byte_order((uint16_t)delta_time);
    memcpy(&payload_buffer[2],&temp_buffer,2);

    /* Create the Sick LD send/receive message objects */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;
  
    /* Send the message and check for the expected reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
            
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickTimeRelative: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,4);

    /* Get the message payload */
    recv_message.GetPayload(payload_buffer);
  
    /* Extract the new Sick LD clock time from the response */
    uint16_t clock_time;
    memcpy(&clock_time,&payload_buffer[2],2);
    new_sick_clock_time = sick_ld_to_host_byte_order(clock_time);

    std::cout << "\t\tClock time set!" << std::endl;
  
    /* Success */
  }

  /**
   * \brief Sets the Sick LD signal LED's and switching outputs
   * \param sick_signal_flags Indicates the LEDs and switches to set/unset.
   *
   * NOTE: This method does not preserve the previous state of the Sick's signals.
   *       In other words, only the signals flagged in sick_signal_flags will be
   *       set - all others will be off!
   */
  void SickLD::SetSickSignals( const uint8_t sick_signal_flags )
    throw( SickIOException, SickTimeoutException, SickErrorException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLD::SetSickSignals: Device NOT Initialized!!!");
    }

    /* Attempt to set the signal flags */
    try {
      _setSickSignals(sick_signal_flags);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_syncDriverWithSick: Unknown exception!!!" << std::endl;
      throw;
    } 

    /* Success! */
  }

  /**
   * \brief Gets the Sick LD signal LED's and switching outputs.
   * \param &sick_signal_flags The destination buffer to hold the returned flags.
   */
  void SickLD::GetSickSignals( uint8_t &sick_signal_flags ) throw( SickIOException, SickTimeoutException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::GetSickSignals: Device NOT Initialized!!!");
    }
  
    /* Initialize the destination buffer */
    sick_signal_flags = 0;
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_STAT_SERV_CODE;       // Requested service type
    payload_buffer[1] = SICK_STAT_SERV_GET_SIGNAL; // Requested service subtype
  
    /* Create the Sick message */
    SickLDMessage send_message(payload_buffer,2);
    SickLDMessage recv_message;
  
    /* Send the message and get the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::GetSickSignals: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the payload buffer */
    memset(payload_buffer,0,2);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract the Signal flags */
    sick_signal_flags = payload_buffer[3];
  
    /* Success */
  }

  /**
   * \brief Gets the internal clock time of the Sick LD unit.
   * \param &sick_time The sick clock time in milliseconds.
   */
  void SickLD::GetSickTime( uint16_t &sick_time )
    throw( SickIOException, SickTimeoutException, SickErrorException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::GetSickTime: Device NOT Initialized!!!");
    }
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;             // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_GET_SYNC_CLOCK;   // Requested service subtype
  
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,2);
    SickLDMessage recv_message;

    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::GetSickTime: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,2);
  
    /* Acquire the returned payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract actual time */
    uint16_t current_time;
    memcpy(&current_time,&payload_buffer[2],2);
    sick_time = sick_ld_to_host_byte_order(current_time);

    /* Success */
  }

  /**
   * \brief Enables nearfield suppressive filtering
   *
   * NOTE: This method writes this option to the Sick LD's flash, so there is no
   *       need to use it except when configuring the device.
   */ 
  void SickLD::EnableNearfieldSuppression( )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device has been initialized */
    if(!_sick_initialized) {
      throw SickIOException("SickLD::EnableNearfieldSuppression: Device NOT Initialized!!!");
    }
  
    /* Tell the Sick LD to use nearfield suppression! */
    std::cout << "\tEnabling nearfield suppression..." << std::endl;
    try {
      _setSickFilter(SICK_CONF_SERV_SET_FILTER_NEARFIELD_ON);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::EnableNearfieldSuppression: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    std::cout << "\t\tSuppression is enabled!" << std::endl;
  
    /* Success */
  }

  /**
   * \brief Disables nearfield suppressive filtering
   *
   * NOTE: This method writes this option to the Sick LD's flash, so there is no
   *       need to use it except when configuring the device.
   */ 
  void SickLD::DisableNearfieldSuppression( )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::DisableNearfieldSuppression: Device NOT Initialized!!!");
    }
  
    /* Tell the Sick LD to use nearfield suppression! */
    std::cout << "\tDisabling nearfield suppression..." << std::endl;
    try {
      _setSickFilter(SICK_CONF_SERV_SET_FILTER_NEARFIELD_OFF);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::DisableNearfieldSuppression: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    std::cout << "\t\tSuppression is disabled!" << std::endl;
  
    /* Success */
  }

  /**
   * \brief Acquires measurements and corresponding sector data from the Sick LD.
   * \param *range_measurements      A single array to hold ALL RANGE MEASUREMENTS from the current scan for all active
   *                                 sectors. Range values from each sector are stored block sequentially in this buffer.
   *                                 The respective index into the array marking the first data value returned by active
   *                                 sector i can be found by getting: range_measurement[sector_data_offsets[i]].
   * \param *echo_measurements       A single array to hold ALL ECHO MEASUREMENTS from the current scan for all active sectors.
   *                                 It is indexed the same as range_measurements. This argument is optional and if it is not
   *                                 provided the driver will request a RANGE ONLY data stream as opposed to a RANGE+ECHO thereby
   *                                 reducing consumed bandwidth (Default: NULL).
   * \param *num_measurements        An array where the ith element denotes the number of range/echo measurements obtained from
   *                                 active sector i (Default: NULL).
   * \param *sector_ids              An array where the ith element corresponds to the actual sector number/id of the ith active
   *                                 sector. (Default: NULL)
   * \param *sector_data_offsets     The index offsets mapping the ith active sector to the position in range_measurements
   *                                 where its first measured value can be found (Default: NULL).
   * \param *sector_step_angles      An array where the ith element corresponds to the angle step for the ith active sector.
   *                                 (Default: NULL)
   * \param *sector_start_angles     An array where the ith element corresponds to the starting scan angle of the ith active
   *                                 sector.
   * \param *sector_stop_angles      An array where the ith element corresponds to the stop scan angle of the ith active sector.
   * \param *sector_start_timestamps An array where the ith element denotes the time at which the first scan was taken for
   *                                 the ith active sector.
   * \param *sector_stop_timestamps  An array where the ith element denotes the time at which the last scan was taken for
   *                                 the ith active sector.
   *
   * ALERT: The user is responsible for ensuring that enough space is allocated for the return buffers to avoid overflow.
   *        See the example code for an easy way to do this.
   */
  void SickLD::GetSickMeasurements( double * const range_measurements,
				    unsigned int * const echo_measurements,
				    unsigned int * const num_measurements,
				    unsigned int * const sector_ids,
				    unsigned int * const sector_data_offsets,
				    double * const sector_step_angles,
				    double * const sector_start_angles,
				    double * const sector_stop_angles,
				    unsigned int * const sector_start_timestamps,
				    unsigned int * const sector_stop_timestamps )
    throw( SickErrorException, SickIOException, SickTimeoutException, SickConfigException ){

    /* Ensure the device has been initialized */
    if(!_sick_initialized) {
      throw SickIOException("SickLD::GetSickMeasurements: Device NOT Initialized!!!");
    }
  
    /* The following conditional holds true if the user wants a RANGE+ECHO data
     * stream but already has an active RANGE-ONLY stream.
     */
    if (_sick_streaming_range_data && echo_measurements != NULL) {

      try {
	
        /* Cancel the current RANGE-ONLY data stream */
        _cancelSickScanProfiles();

        /* Request a RANGE+ECHO data stream */
        _getSickScanProfiles(SICK_SCAN_PROFILE_RANGE_AND_ECHO);

      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::GetSickMeasurements: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }

    /* The following conditional holds true if the user wants a RANGE-ONLY data
     * stream but already has an active RANGE+ECHO stream.
     */
    if (_sick_streaming_range_and_echo_data && echo_measurements == NULL) {

      try {

	/* Cancel the current RANGE+ECHO data stream */
        _cancelSickScanProfiles();

        /* Request a RANGE-ONLY data stream */
        _getSickScanProfiles(SICK_SCAN_PROFILE_RANGE);
	
      }
      
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::GetSickMeasurements: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }

    /* If there aren't any active data streams, setup a new one */
    if (!_sick_streaming_range_data && !_sick_streaming_range_and_echo_data) {

      try {
      
	/* Determine the target data stream by checking the value of echo_measurements */
	if (echo_measurements != NULL) {
	  
	  /* Request a RANGE+ECHO data stream */
	  _getSickScanProfiles(SICK_SCAN_PROFILE_RANGE_AND_ECHO);	  
	  
	}
	else {
	  
	  /* Request a RANGE+ONLY data stream */
	  _getSickScanProfiles(SICK_SCAN_PROFILE_RANGE);
	  
	}

      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }

    /* Declare the receive message object */
    SickLDMessage recv_message;
  
    /* Acquire the most recently buffered message */
    try {
      _recvMessage(recv_message,(unsigned int)1e6);
    }
    
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }  

    catch(...) {
      std::cerr << "SickLD::GetSickMeasurements - Unknown exception!" << std::endl;
      throw;
    }
    
    /* A single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Get the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Define the destination Sick LD scan profile struct */
    sick_ld_scan_profile_t profile_data;
  
    /* Extract the scan profile */
    _parseScanProfile(&payload_buffer[2],profile_data);

    /* Update and check the returned sensor status */
    if ((_sick_sensor_mode = profile_data.sensor_status) != SICK_SENSOR_MODE_MEASURE) {
      throw SickConfigException("SickLD::GetSickMeasurements: Unexpected sensor mode! " + _sickSensorModeToString(_sick_sensor_mode));
    }

    /* Update and check the returned motor status */
    if ((_sick_motor_mode = profile_data.motor_status) != SICK_MOTOR_MODE_OK) {
      throw SickConfigException("SickLD::GetSickMeasurements: Unexpected motor mode! (Are you using a valid motor speed!)");
    }

    /* Everything is OK, so now populate the relevant return buffers */
    for (unsigned int i = 0, total_measurements = 0; i < _sick_sector_config.sick_num_active_sectors; i++) {

      /* Copy over the returned range values */
      memcpy(&range_measurements[total_measurements],profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].range_values,
	     profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].num_data_points*sizeof(double));
    
      /* Copy the returned echo values  if requested */
      if (echo_measurements != NULL) {
	memcpy(&echo_measurements[total_measurements],profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].echo_values,
	       profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].num_data_points*sizeof(unsigned int));
      }
    
      /* Set the number of measurements */
      if (num_measurements != NULL) {
	num_measurements[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].num_data_points;
      }
    
      /* Set the associated sector's id if requested */
      if (sector_ids != NULL) {
	sector_ids[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].sector_num;
      }
    
      /* Set the associated sector's index into the range measurement buffer if requested */
      if (sector_data_offsets != NULL) {
	sector_data_offsets[i] = total_measurements;
      }
    
      /* Set the step angle if requested */
      if (sector_step_angles != NULL) {
	sector_step_angles[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].angle_step;
      }
    
      /* Set the sector start angle if requested */
      if (sector_start_angles != NULL) {
	sector_start_angles[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].angle_start;
      }
    
      /* Set the sector stop angle if requested */
      if (sector_stop_angles != NULL) {
	sector_stop_angles[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].angle_stop;
      }
    
      /* Set the sector start timestamp if requested */
      if (sector_start_timestamps != NULL) {
	sector_start_timestamps[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].timestamp_start;
      }
    
      /* Set the sector stop timestamp if requested */
      if (sector_stop_timestamps != NULL) {
	sector_stop_timestamps[i] = profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].timestamp_stop;
      }
    
      /* Update the total number of measurements */
      total_measurements += profile_data.sector_data[_sick_sector_config.sick_active_sector_ids[i]].num_data_points;
    }

    /* Success */
  
  }

  /**
   * \brief Attempts to set a new sensor ID for the device (in flash)
   * \param sick_sensor_id The desired sensor ID
   */
  void SickLD::SetSickSensorID( const unsigned int sick_sensor_id )
    throw( SickErrorException, SickTimeoutException, SickIOException ){

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLD::SetSickSensorID: Device NOT Initialized!!!");
    }

    /* Check that the given ID is valid */
    if (!_validSickSensorID(sick_sensor_id)) {
      throw SickConfigException("SickLD::SetSickSensorID: Invalid sensor ID!!!");
    }

    /* Attempt to set the new sensor ID in the flash! */
    try {
      _setSickGlobalConfig(sick_sensor_id,GetSickMotorSpeed(),GetSickScanResolution());
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::SetSickSensorID: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* Success! */
  }

  /**
   * \brief Attempts to set a new motor speed for the device (in flash)
   * \param sick_motor_speed The desired motor speed (Hz)
   */
  void SickLD::SetSickMotorSpeed( const unsigned int sick_motor_speed )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::SetSickMotorSpeed: Device NOT Initialized!!!");
    }

    /* Check that the given motor speed is valid */
    if (!_validSickMotorSpeed(sick_motor_speed)) {
      throw SickConfigException("SickLD::SetSickMotorSpeed: Invalid sick motor speed!!!");
    }
    
    /* Check to ensure a valid pulse frequency for the device */
    if (!_validPulseFrequency(sick_motor_speed,GetSickScanResolution())) {
      throw SickConfigException("SickLD::SetSickMotorSpeed: Invalid pulse frequency!!!");
    }
    
    /* Attempt to set the new global config in the flash! */
    try {
      _setSickGlobalConfig(GetSickSensorID(),sick_motor_speed,GetSickScanResolution());
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::SetSickMotorSpeed: Unknown exception!!!" << std::endl;
      throw;
    }      
  
    /* Success! */
  }

  /**
   * \brief Attempts to set a new scan resolution for the device (in flash) while
   *        retaining the previously defined active scan areas.
   * \param sick_angle_step The desired scan resolution (deg)
   *
   * NOTE: Since the Sick determines the active sector bounds based upon the current
   *       scan resolution and sector stop angles, it is possible that when the scan
   *       resolution is changed so too are the active sector bounds.  As such, we
   *       also write the correct sector config to flash as opposed to letting the
   *       Sick LD just figure it out itself.  Doing so ensures the previous active
   *       sector definitions remain intact.
   */
  void SickLD::SetSickScanResolution( const double sick_angle_step )
    throw( SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::SetSickScanResolution: Device NOT Initialized!!!");
    }
  
    /* Buffers to hold the active sector data */
    double active_sector_start_angles[SICK_MAX_NUM_SECTORS] = {0};
    double active_sector_stop_angles[SICK_MAX_NUM_SECTORS] = {0};
  
    /* Extract the start and stop angles for the current active sectors */
    for (unsigned int i = 0; i < _sick_sector_config.sick_num_active_sectors; i++) {
      active_sector_start_angles[i] = _sick_sector_config.sick_sector_start_angles[_sick_sector_config.sick_active_sector_ids[i]];
      active_sector_stop_angles[i] = _sick_sector_config.sick_sector_stop_angles[_sick_sector_config.sick_active_sector_ids[i]];   
    }

    /* Set the operating parameters accordingly */
    try { 
      SetSickGlobalParamsAndScanAreas(GetSickMotorSpeed(),sick_angle_step,
					 active_sector_start_angles,active_sector_stop_angles,
					 _sick_sector_config.sick_num_active_sectors);
    }

    catch(...) { }
  
    /* Success! */
  }

  /**
   * \brief Attempts to set the scan resolution and active sectors/scan areas for the device (in flash)
   * \param sick_motor_speed Desired sick motor speed (Hz)
   * \param sick_angle_step Desired scan angular resolution (deg)
   * \param active_sector_start_angles Angles marking the beginning of each desired active sector/area
   * \param active_sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_active_sectors The number of active sectors/scan areas are given
   */
  void SickLD::SetSickGlobalParamsAndScanAreas( const unsigned int sick_motor_speed,
						const double sick_angle_step,
						const double * const active_sector_start_angles,
						const double * const active_sector_stop_angles,
						const unsigned int num_active_sectors )
    throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::SetSickGlobalParamsAndScanAreas: Device NOT Initialized!!!");
    }
  
    /* Attempt to write both a new scan resolution and scan area config */
    try {
      _setSickGlobalParamsAndScanAreas(sick_motor_speed,sick_angle_step,
				       active_sector_start_angles,active_sector_stop_angles,num_active_sectors);    
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle config exception */
    catch (SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::SetSickGlobalParamsAndScanAreas: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* Success! */
  
  }

  /**
   * \brief Attempts to set the active scan areas for the device (in flash)
   * \param active_sector_start_angles Angles marking the beginning of each desired active sector/area
   * \param active_sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_active_sectors The number of active sectors/scan areas are given
   */
  void SickLD::SetSickScanAreas( const double * const active_sector_start_angles,
				 const double * const active_sector_stop_angles,
				 const unsigned int num_active_sectors )
    throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::SetSickScanAreas: Device NOT Initialized!!!");
    }
  
    /* Attempt to write both a new scan resolution and scan area config */
    try {
      _setSickGlobalParamsAndScanAreas(GetSickMotorSpeed(),GetSickScanResolution(),
					  active_sector_start_angles,active_sector_stop_angles,num_active_sectors);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle config exception */
    catch (SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::SetSickScanAreas: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    /* Success! */
  
  }

  /**
   * \brief Resets the device according to the given reset level
   * \param reset_level The desired reset level (see page 33 of the telegram listing)
   */
  void SickLD::ResetSick( const unsigned int reset_level )
    throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::ResetSick: Device NOT Initialized!!!");
    }
  
    /* Ensure a valid reset level was given */
    if (reset_level > SICK_WORK_SERV_RESET_HALT_APP) {
      throw SickConfigException("SickLD::ResetSick: Invalid given reset level!");
    }
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_WORK_SERV_CODE;             // Requested service type
    payload_buffer[1] = SICK_WORK_SERV_RESET;            // Requested service subtype
    payload_buffer[3] = (uint8_t)reset_level;            // RESETLEVEL
  
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;

    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,4);
  
    /* Acquire the returned payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract the returned reset level */
    uint16_t current_reset_level;
    memcpy(&current_reset_level,&payload_buffer[2],2);
    current_reset_level = sick_ld_to_host_byte_order(current_reset_level);

    /* Verify the returned reset level */
    if (current_reset_level != (uint16_t)reset_level) {
      throw SickErrorException("SickLD::ResetSick: Unexpected returned reset level!");
    }

    /* Success */
  }

  /**
   * \brief Acquire the number of sectors that are measuring
   * \return The number of sectors currently measuring
   */
  unsigned int SickLD::GetSickNumActiveSectors( ) const {
    return _sick_sector_config.sick_num_active_sectors;
  }

  /**
   * \brief Acquire the Sick LD's sensor ID
   * \return The Sick LD sensor ID
   */
  unsigned int SickLD::GetSickSensorID( ) const {
    return _sick_global_config.sick_sensor_id;
  }

  /**
   * \brief Acquire the Sick LD's current motor speed in Hz
   * \return The Sick LD motor speed
   */
  unsigned int SickLD::GetSickMotorSpeed( ) const {
    return _sick_global_config.sick_motor_speed;
  }

  /**
   * \brief Acquire the Sick LD's current scan resolution
   * \return The Sick LD scan resolution
   */
  double SickLD::GetSickScanResolution( ) const {
    return _sick_global_config.sick_angle_step;
  }

  /**
   * \brief Acquire the current IP address of the Sick
   * \return The Sick LD IP (Inet4) address
   */
  std::string SickLD::GetSickIPAddress( ) const {

    /* Declare the string stream */
    std::ostringstream str_stream;

    str_stream << _sick_ethernet_config.sick_ip_address[0] << "."
	       << _sick_ethernet_config.sick_ip_address[1] << "."
	       << _sick_ethernet_config.sick_ip_address[2] << "."
	       << _sick_ethernet_config.sick_ip_address[3];

    /* Return the std string representation */
    return str_stream.str();

  }

  /**
   * \brief Acquire the subnet mask for the Sick
   * \return The Sick LD subnet mask
   */
  std::string SickLD::GetSickSubnetMask( ) const {

    /* Declare the string stream */
    std::ostringstream str_stream;

    str_stream << _sick_ethernet_config.sick_subnet_mask[0] << "."
	       << _sick_ethernet_config.sick_subnet_mask[1] << "."
	       << _sick_ethernet_config.sick_subnet_mask[2] << "."
	       << _sick_ethernet_config.sick_subnet_mask[3];

    /* Return the std string representation */
    return str_stream.str();
 
  }

  /**
   * \brief Acquire the IP address of the Sick gateway
   * \return The Sick LD gateway IP address
   */
  std::string SickLD::GetSickGatewayIPAddress( ) const {

    /* Declare the string stream */
    std::ostringstream str_stream;

    str_stream << _sick_ethernet_config.sick_gateway_ip_address[0] << "."
	       << _sick_ethernet_config.sick_gateway_ip_address[1] << "."
	       << _sick_ethernet_config.sick_gateway_ip_address[2] << "."
	       << _sick_ethernet_config.sick_gateway_ip_address[3];

    /* Return the std string representation */
    return str_stream.str();

  }

  /**
   * \brief Acquire the Sick LD's part number
   * \return The Sick LD part number
   */
  std::string SickLD::GetSickPartNumber( ) const {
    return _sick_identity.sick_part_number;
  }

  /**
   * \brief Acquire the Sick LD's name
   * \return The Sick LD sensor name
   */
  std::string SickLD::GetSickName( ) const {
    return _sick_identity.sick_name;
  }

  /**
   * \brief Acquire the Sick LD's version number
   * \return The Sick LD version number
   */
  std::string SickLD::GetSickVersion( ) const {
    return _sick_identity.sick_version;
  }

  /**
   * \brief Acquire the Sick LD's serial number
   * \return The Sick LD serial number
   */
  std::string SickLD::GetSickSerialNumber( ) const {
    return _sick_identity.sick_serial_number;
  }

  /**
   * \brief Acquire the Sick LD's EDM serial number
   * \return The Sick LD EDM serial number
   */
  std::string SickLD::GetSickEDMSerialNumber( ) const {
    return _sick_identity.sick_edm_serial_number;
  }

  /**
   * \brief Acquire the Sick LD's firmware part number
   * \return The Sick LD firmware part number
   */
  std::string SickLD::GetSickFirmwarePartNumber( ) const {
    return _sick_identity.sick_firmware_part_number;
  }

  /**
   * \brief Acquire the Sick LD's firmware number
   * \return The Sick LD firmware name
   */
  std::string SickLD::GetSickFirmwareName( ) const {
    return _sick_identity.sick_firmware_name;
  }

  /**
   * \brief Acquire the Sick LD's firmware version
   * \return The Sick LD firmware version
   */
  std::string SickLD::GetSickFirmwareVersion( ) const {
    return _sick_identity.sick_firmware_version;
  }

  /**
   * \brief Acquire the Sick LD's application software part number
   * \return The Sick LD application software part number
   */
  std::string SickLD::GetSickAppSoftwarePartNumber( ) const {
    return _sick_identity.sick_application_software_part_number;
  }

  /**
   * \brief Acquire the Sick LD's application software name
   * \return The Sick LD application software name
   */
  std::string SickLD::GetSickAppSoftwareName( ) const {
    return _sick_identity.sick_application_software_name;
  }

  /**
   * \brief Acquire the Sick LD's application software version number
   * \return The Sick LD application software version number
   */
  std::string SickLD::GetSickAppSoftwareVersionNumber( ) const {
    return _sick_identity.sick_application_software_version;
  }

  /**
   * \brief Acquire the Sick LD's status as a printable string
   * \return The Sick LD status as a well-formatted string
   */
  std::string SickLD::GetSickStatusAsString() const {

    std::stringstream str_stream;

    str_stream << "\t============= Sick LD Status =============" << std::endl;
    str_stream << "\tSensor Mode: " << _sickSensorModeToString(_sick_sensor_mode) << std::endl;
    str_stream << "\tMotor Mode: " << _sickMotorModeToString(_sick_motor_mode) << std::endl;
    str_stream << "\t==========================================" << std::endl;    

    return str_stream.str();
    
  }
  
  /**
   * \brief Acquire the Sick LD's identity as a printable string
   * \return The Sick LD identity as a well-formatted string
   */
  std::string SickLD::GetSickIdentityAsString() const {

    std::ostringstream str_stream;
    
    str_stream << "\t============ Sick LD Identity ============" << std::endl;
    str_stream << "\tSensor Part #: " << GetSickPartNumber() << std::endl;
    str_stream << "\tSensor Name: " << GetSickName() << std::endl;
    str_stream << "\tSensor Version: " << GetSickVersion() << std::endl;
    str_stream << "\tSensor Serial #: " << GetSickSerialNumber() << std::endl;
    str_stream << "\tSensor EDM Serial #: " << GetSickEDMSerialNumber() << std::endl;
    str_stream << "\tFirmware Part #: " << GetSickFirmwarePartNumber() << std::endl;
    str_stream << "\tFirmware Version: " << GetSickFirmwareVersion() << std::endl;
    str_stream << "\tFirmware Name: " << GetSickFirmwareName() << std::endl;
    str_stream << "\tApp. Software Part #: " << GetSickAppSoftwarePartNumber() << std::endl;
    str_stream << "\tApp. Software Name: " << GetSickAppSoftwareName() << std::endl;
    str_stream << "\tApp. Software Version: " << GetSickAppSoftwareVersionNumber() << std::endl;
    str_stream << "\t==========================================" << std::endl;
    
    return str_stream.str();

  }
  
  /**
   * \brief Acquire the Sick LD's global config as a printable string
   * \return The Sick LD global config as a well-formatted string
   */
  std::string SickLD::GetSickGlobalConfigAsString() const {

    std::stringstream str_stream;
    
    str_stream << "\t=========== Sick Global Config ===========" << std::endl;
    str_stream << "\tSensor ID: " << GetSickSensorID() << std::endl;
    str_stream << "\tMotor Speed (5 to 20Hz): " << GetSickMotorSpeed() << std::endl;
    str_stream << "\tAngle Step (deg): " << GetSickScanResolution() << std::endl;
    str_stream << "\t==========================================" << std::endl;
    
    return str_stream.str();
    
  }

  /**
   * \brief Acquire the Sick LD's global config as a printable string
   * \return The Sick LD Ethernet config as a well-formatted string
   */
  std::string SickLD::GetSickEthernetConfigAsString() const {

    std::stringstream str_stream;

    str_stream << "\t========== Sick Ethernet Config ==========" << std::endl;
    str_stream << "\tIP Address: " << GetSickIPAddress() << std::endl;
    str_stream << "\tSubnet Mask: " << GetSickSubnetMask() << std::endl;
    str_stream << "\tGateway IP Address: " << GetSickGatewayIPAddress() << std::endl;
    str_stream << "\t==========================================" << std::endl;
    
    return str_stream.str();
    
  }
  
  /**
   * \brief Acquire the Sick LD's sector config as a printable string
   * \return The Sick LD Sick LD's config as a well-formatted string
   */
  std::string SickLD::GetSickSectorConfigAsString() const {

    std::stringstream str_stream;
    
    str_stream << "\t=========== Sick Sector Config ===========" << std::endl;
    str_stream << "\tNum. Active Sectors: " << (int)_sick_sector_config.sick_num_active_sectors << std::endl;
    str_stream << "\tNum. Initialized Sectors: " << (int)_sick_sector_config.sick_num_initialized_sectors << std::endl;
    
    str_stream << "\tSector Configs.:" << std::endl;
    for (unsigned int i = 0; i < _sick_sector_config.sick_num_initialized_sectors; i++) {
      str_stream << "\t\t" << i << " ["
		 << _sick_sector_config.sick_sector_start_angles[i] << ","
		 << _sick_sector_config.sick_sector_stop_angles[i] << "] ("
		 << _sickSectorFunctionToString(_sick_sector_config.sick_sector_functions[i]) << ")" << std::endl;
    }
    
    str_stream << "\t==========================================" << std::endl;
    
    return str_stream.str();

  }
  
  /**
   * \brief Computes the active area over all measuring sectors
   * \return The Sick LD active scan area
   */
  double SickLD::GetSickScanArea( ) const {

    /* Some temp buffers */
    double sector_start_angles[SICK_MAX_NUM_SECTORS] = {0};
    double sector_stop_angles[SICK_MAX_NUM_SECTORS] = {0};
  
    /* Sum the active areas over all sectors */
    for (unsigned int i = 0; i < _sick_sector_config.sick_num_active_sectors; i++) {
      sector_start_angles[i] = _sick_sector_config.sick_sector_start_angles[_sick_sector_config.sick_active_sector_ids[i]];
      sector_stop_angles[i] = _sick_sector_config.sick_sector_stop_angles[_sick_sector_config.sick_active_sector_ids[i]];    
    }  
  
    /* Return the computed total scan area */
    return _computeScanArea(GetSickScanResolution(),sector_start_angles,sector_stop_angles,_sick_sector_config.sick_num_active_sectors);
  }

  /**
   * \brief Print the status of the Sick LD
   */
  void SickLD::PrintSickStatus( ) const {  
    std::cout << GetSickStatusAsString() << std::flush;
  }

  /**
   * \brief Print the parameters comprising the Sick LD's identity
   */
  void SickLD::PrintSickIdentity( ) const {
    std::cout << GetSickIdentityAsString() << std::flush;
  }

  /**
   * \brief Print the Sick LD's global configuration
   */
  void SickLD::PrintSickGlobalConfig( ) const {  
    std::cout << GetSickGlobalConfigAsString() << std::flush;
  }

  /**
   * \brief Print the Sick LD's Ethernet configuration
   */
  void SickLD::PrintSickEthernetConfig( ) const {  
    std::cout << GetSickEthernetConfigAsString() << std::flush;
  }

  /**
   * \brief Print the Sick LD's sector configuration
   */
  void SickLD::PrintSickSectorConfig( ) const {  
    std::cout << GetSickSectorConfigAsString() << std::flush;
  }

  /**
   * \brief Tear down the connection between the host and the Sick LD
   */
  void SickLD::Uninitialize( ) throw( SickIOException, SickTimeoutException, SickErrorException, SickThreadException ){

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLD::Uninitialize: Device NOT Initialized!!!");
    }

    std::cout << std::endl << "\t*** Attempting to uninitialize the Sick LD..." << std::endl; 
  
    /* If necessary, tell the Sick LD to stop streaming data */
    try {
      
      std::cout << "\tSetting Sick LD to idle mode..." << std::endl;
      _setSickSensorModeToIdle();
      std::cout << "\t\tSick LD is now idle!" << std::endl;

      /* Clear any signals that were set */
      SetSickSignals();

      /* Attempt to cancel the buffer monitor */
      std::cout << "\tAttempting to cancel buffer monitor..." << std::endl;
      _stopListening();
      std::cout << "\t\tBuffer monitor canceled!" << std::endl;
    
      /* Attempt to close the tcp connection */
      std::cout << "\tClosing connection to Sick LD..." << std::endl;
      _teardownConnection();

    }
           
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }

    /* Handle a returned error code */
    catch (SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  

    std::cout << "\t\tConnection closed!" << std::endl;

    std::cout << "\t*** Uninit. complete - Sick LD is now offline!" << std::endl; 

    /* Mark the device as uninitialized */
    _sick_initialized = false;
  
    /* Success! */
  }

  /**
   * \brief Establish a TCP connection to the unit
   */
  void SickLD::_setupConnection( ) throw( SickIOException, SickTimeoutException ) {

    /* Create the TCP socket */
    if ((_sick_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0) {
      throw SickIOException("SickLD::_setupConnection: socket() failed!");
    }

    /* Initialize the buffer */
    memset(&_sick_inet_address_info,0,sizeof(struct sockaddr_in));
  
    /* Setup the Sick LD inet address structure */
    _sick_inet_address_info.sin_family = AF_INET;                                  // Internet protocol address family
    _sick_inet_address_info.sin_port = htons(_sick_tcp_port);                      // TCP port number
    _sick_inet_address_info.sin_addr.s_addr = inet_addr(_sick_ip_address.c_str()); // Convert ip string to numerical address

    try {

      /* Set to non-blocking so we can time connect */
      _setNonBlockingIO();
    
      /* Try to connect to the Sick LD */
      int conn_return;
      if ((conn_return = connect(_sick_fd,(struct sockaddr *)&_sick_inet_address_info,sizeof(struct sockaddr_in))) < 0) {

	/* Check whether it is b/c it would block */
	if (errno != EINPROGRESS) {	
	  throw SickIOException("SickLD::_setupConnection: connect() failed!");
	}

	/* Use select to wait on the socket */
	int valid_opt = 0;
	int num_active_files = 0;
	struct timeval timeout_val;                          // This structure will be used for setting our timeout values
	fd_set file_desc_set;                                // File descriptor set for monitoring I/O
    
	/* Initialize and set the file descriptor set for select */
	FD_ZERO(&file_desc_set);
	FD_SET(_sick_fd,&file_desc_set);

	/* Setup the timeout structure */
	memset(&timeout_val,0,sizeof(timeout_val));          // Initialize the buffer
	timeout_val.tv_usec = DEFAULT_SICK_CONNECT_TIMEOUT;  // Wait for specified time before throwing a timeout
      
	/* Wait for the OS to tell us that the connection is established! */
	num_active_files = select(getdtablesize(),0,&file_desc_set,0,&timeout_val);
      
	/* Figure out what to do based on the output of select */
	if (num_active_files > 0) {
	
	  /* This is just a sanity check */
	  if (!FD_ISSET(_sick_fd,&file_desc_set)) {
  	    throw SickIOException("SickLD::_setupConnection: Unexpected file descriptor!");
	  }	  

	  /* Check for any errors on the socket - just to be sure */
	  socklen_t len = sizeof(int);
	  if (getsockopt(_sick_fd,SOL_SOCKET,SO_ERROR,(void*)(&valid_opt),&len) < 0) { 	    
  	    throw SickIOException("SickLD::_setupConnection: getsockopt() failed!");
	  } 

	  /* Check whether the opt value indicates error */
	  if (valid_opt) { 
	    throw SickIOException("SickLD::_setupConnection: socket error on connect()!");
	  }
	  
  	}
	else if (num_active_files == 0) {
	
	  /* A timeout has occurred! */
	  throw SickTimeoutException("SickLD::_setupConnection: select() timeout!");	

	}
	else {
	
	  /* An error has occurred! */
	  throw SickIOException("SickLD::_setupConnection: select() failed!");	

	}

      }

      /* Restore blocking IO */
      _setBlockingIO();	
	
    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    catch(...) {
      std::cerr << "SickLD::_setupConnection - Unknown exception occurred!" << std::endl;
      throw;
    }

    /* Success */
  }

  /**
   * \brief Synchronizes buffer driver parameters with those of the Sick LD.
   *
   * \todo Add support in this method for acquiring RS-232/RS-422, and CAN configurations
   */
  void SickLD::_syncDriverWithSick( )  throw( SickIOException, SickTimeoutException, SickErrorException ) {

    try {
      
      /* Acquire current configuration */
      _getSickStatus();
      _getSickIdentity();
      _getSickEthernetConfig();
      _getSickGlobalConfig();
      _getSickSectorConfig();

      /* Reset Sick signals */
      _setSickSignals();

    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_syncDriverWithSick: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* Success */
  }

  /** \brief Sets the function for a particular scan sector.
   *  \param sector_number The number of the sector (should be in [0,7])
   *  \param sector_function The function of the sector (e.g. no measurement, reserved, normal measurement, ...)
   *  \param sector_stop_angle The last angle of the sector (in odometer ticks)
   *  \param write_to_flash Indicates whether the sector configuration should be written to flash
   */
  void SickLD::_setSickSectorFunction ( const uint8_t sector_number, const uint8_t sector_function,
					const double sector_stop_angle, const bool write_to_flash )
    throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device is not measuring */
    if (_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) {

      /* Set the Sick LD to rotate mode */
      try {
        _setSickSensorModeToRotate();
      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickSectorFunction: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }
  
    /* Ensure a valid sector number */
    if (sector_number >= SICK_MAX_NUM_SECTORS) {
      throw SickConfigException("SickLD::_setSickSectorFunction: Invalid sector number!");
    }

    /* Check that a valid sector_function was given */
    if (sector_function != SICK_CONF_SECTOR_NOT_INITIALIZED &&
        sector_function != SICK_CONF_SECTOR_NO_MEASUREMENT &&
        sector_function != SICK_CONF_SECTOR_RESERVED &&
        sector_function != SICK_CONF_SECTOR_NORMAL_MEASUREMENT &&
        sector_function != SICK_CONF_SECTOR_REFERENCE_MEASUREMENT) {

      throw SickConfigException("SickLD::_setSickSectorFunction: Invalid sector function code!");
    }

    /* Check that a valid stop angle was given */
    if (sector_stop_angle > SICK_MAX_SCAN_AREA) {
      throw SickConfigException("SickLD::_setSickSectorFunction: Invalid sector stop angle!");
    }
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* A temporary buffer for byte order conversion */
    uint16_t temp_buff = 0;
  
    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;          // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_SET_FUNCTION;  // Requested service subtype                  

    /* Assign the payload data */
    payload_buffer[3] = sector_number;                // SECTORNUM
    payload_buffer[5] = sector_function;              // SECTORFUNC

    /* Set the sector stop value */
    temp_buff = host_to_sick_ld_byte_order(_angleToTicks(sector_stop_angle));    
    memcpy(&payload_buffer[6],&temp_buff,2);          // SECTORSTOP

    /* Include the flash flag */
    payload_buffer[9] = (uint8_t)write_to_flash;      // FLASHFLAG

    /* Create the Sick LD messages */
    SickLDMessage send_message(payload_buffer,10);    
    SickLDMessage recv_message;

    /* Send the message and get the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSectorFunction: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer (not necessary, but it doesn't hurt to be careful) */
    memset(payload_buffer,0,10);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);  

    /* Check the response for an error */
    if (payload_buffer[2] == 0xFF && payload_buffer[3] == 0xFF) {
      throw SickConfigException("SickLD::_setSickSectorFunction: Invalid request!");
    }
  
    /* Success! */
  }

  /**
   * \brief Acquires the function of the given sector
   * \param sector_num The target sector number
   * \param &sector_config The configuration word returned by the Sick LD
   * \param &sector_stop The stop angle of the given sector
   */
  void SickLD::_getSickSectorFunction( const uint8_t sector_num, uint8_t &sector_function, double &sector_stop_angle )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device is not measuring */
    if (_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) {

      /* Set the Sick LD to rotate mode */
      try {
        _setSickSensorModeToRotate();
      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_getSickSectorFunction: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }
  
    /* Declare the message payload buffer */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;            // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_GET_FUNCTION;    // Requested service subtype
    payload_buffer[3] = sector_num;                     // Sector number
    
    /* Declare the send/recv Sick LD message objects */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;

    /* Send the message and get a response */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
            
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSickSectorFunction: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,4);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract the returned sector number */
    uint16_t temp_buffer = 0;
    memcpy(&temp_buffer,&payload_buffer[2],2);
    temp_buffer = sick_ld_to_host_byte_order(temp_buffer);

    /* Check to make sure the returned sector number matches
     * the requested sector number.
     */
    if (temp_buffer != sector_num) {
      throw SickConfigException("SickLD::_getSickSectorFunction: Unexpected sector number returned by Sick LD!");
    }

    /* Extract the sector function */
    memcpy(&temp_buffer,&payload_buffer[4],2);
    sector_function = sick_ld_to_host_byte_order(temp_buffer);

    /* Extract the sector stop angle (in ticks) */
    memcpy(&temp_buffer,&payload_buffer[6],2);
    sector_stop_angle = _ticksToAngle(sick_ld_to_host_byte_order(temp_buffer));
  
    /* S'ok */
  }

  /**
   * \brief Sets the Sick LD sensor mode to IDLE
   */
  void SickLD::_setSickSensorModeToIdle( )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* If necessary adjust the operating mode of the sensor */
    if (_sick_sensor_mode != SICK_SENSOR_MODE_IDLE) {
    
      /* Switch the sensor's operating mode to IDLE */
      try {
        _setSickSensorMode(SICK_SENSOR_MODE_IDLE);
      }
      
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }

      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickSensorModeToIdle: Unknown exception!!!" << std::endl;
	throw;
      }  
      
    }
    
    /* Success */
  }

  /**
   * \brief Sets the Sick LD sensor mode to ROTATE
   */
  void SickLD::_setSickSensorModeToRotate( )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* If necessary adjust the operating mode of the sensor */
    if (_sick_sensor_mode != SICK_SENSOR_MODE_ROTATE) {
    
      /* Switch the sensor's operating mode to ROTATE */
      try {
        _setSickSensorMode(SICK_SENSOR_MODE_ROTATE);
      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }

      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickSensorModeToRotate: Unknown exception!!!" << std::endl;
	throw;
      }  
    
    }
  
    /* Success */
  }

  /**
   * \brief Sets the Sick LD sensor mode to ROTATE
   */
  void SickLD::_setSickSensorModeToMeasure( ) 
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* If necessary adjust the operating mode of the sensor */
    if (_sick_sensor_mode != SICK_SENSOR_MODE_MEASURE) {
    
      /* Switch the sensor's operating mode to MEASURE */
      try {
        _setSickSensorMode(SICK_SENSOR_MODE_MEASURE);
      }

      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }

      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickSensorModeToMeasure: Unknown exception!!!" << std::endl;
	throw;
      }  
    
    }
  
    /* Success */
  }

  /**
   * \brief Sets the Sick LD to the requested sensor mode
   * \param new_sick_sensor_mode The desired sensor mode
   */
  void SickLD::_setSickSensorMode( const uint8_t new_sick_sensor_mode ) 
    throw( SickErrorException, SickTimeoutException, SickIOException ) {
  
    /* If the new mode matches the current mode then just return */
    if (_sick_sensor_mode == new_sick_sensor_mode) {
      return;
    }

    try {
    
      /* If the current sensor mode is MEASURE and streaming data */
      if ((_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) &&
	  (_sick_streaming_range_data || _sick_streaming_range_and_echo_data)) {
	
	/* Cancel the current stream */
	_cancelSickScanProfiles();
	
      }
      
      /* The Sick LD must be in rotate mode before: going from IDLE to MEASURE or going from MEASURE to IDLE */
      if ((_sick_sensor_mode == SICK_SENSOR_MODE_IDLE && new_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) ||
	  (_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE && new_sick_sensor_mode == SICK_SENSOR_MODE_IDLE)) {
	
	/* Set to rotate mode */
	_setSickSensorModeToRotate();
	
      }

    }
          
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    /* The payload length */
    uint32_t payload_length = 2;
    
    /* Set the service IDs */
    payload_buffer[0] = SICK_WORK_SERV_CODE;                                       // Requested service type
    payload_buffer[1] = _sickSensorModeToWorkServiceSubcode(new_sick_sensor_mode); // Requested service subtype
    
    /* If the target sensor mode is rotate then we add two more bytes
     * to the payload length. Doing so adds two zero values to the payload
     * which tells it to use the angular step and scan freqeuncy values
     * stored in its flash.
     */
    if (new_sick_sensor_mode == SICK_SENSOR_MODE_ROTATE) {
      payload_length += 2;
    }
    
    /* Define the send/receive message objects */
    SickLDMessage send_message(payload_buffer,payload_length);
    SickLDMessage recv_message;

    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }
      
    /* Reset the payload buffer */
    memset(payload_buffer,0,payload_length);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Ensure the returned mode matches the requested mode */
    if ((_sick_sensor_mode = (payload_buffer[5] & 0x0F)) != new_sick_sensor_mode) {

      /* Check whether there is an error code we can use */
      if (new_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) {

	uint16_t return_code = 0;
        std::string errMsg = "SickLD::_setSickSensorMode: Unexpected sensor mode returned from Sick LD!";
	memcpy(&return_code,&payload_buffer[6],2);
	return_code = sick_ld_to_host_byte_order(return_code);

	/* Print the error code associated with the TRANS_MEASURE request */
        errMsg = errMsg + " (TRANS_MEAS Error Code: " + _sickTransMeasureReturnToString(return_code) + ")";
	throw SickErrorException(errMsg.c_str());
	
      }

    }

    /* Make sure the motor is Ok */
    if ((_sick_motor_mode = ((payload_buffer[5] >> 4) & 0x0F)) != SICK_MOTOR_MODE_OK) {
      throw SickErrorException("SickLD::_setSickSensorMode: Unexpected motor mode returned from Sick LD!");
    }

    /* Success */

  }

  /**
   * \brief Request n scan profiles from the Sick LD unit
   * \param profile_format The format for the requested scan profiles
   * \param num_profiles The number of profiles to request from Sick LD. (Default: 0)
   *                     (NOTE: When num_profiles = 0, the Sick LD continuously streams profile data)
   */
  void SickLD::_getSickScanProfiles( const uint16_t profile_format, const uint16_t num_profiles )
    throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException ) {

    /* Ensure the device is in measurement mode */
    try {
      _setSickSensorModeToMeasure();
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* A quick check to ensure the requested format is supported by the driver */
    if (!_supportedScanProfileFormat(profile_format)) {
      throw SickConfigException("SickLD::_getSickScanProfiles: Unsupported profile format!");
    }
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service code and subcode */
    payload_buffer[0] = SICK_MEAS_SERV_CODE;
    payload_buffer[1] = SICK_MEAS_SERV_GET_PROFILE;

    /* Write the number of profiles to request to the payload buffer */
    uint16_t temp_buffer = host_to_sick_ld_byte_order(num_profiles);
    memcpy(&payload_buffer[2],&temp_buffer,2);

    /* Set the profile format mask (for now, we request everything from the Sick LD) */
    temp_buffer = profile_format;
    temp_buffer = host_to_sick_ld_byte_order(temp_buffer);
    memcpy(&payload_buffer[4],&temp_buffer,2);
  
    /* Define the send message object */
    SickLDMessage send_message(payload_buffer,6);
    SickLDMessage recv_message;
  
    /* Send the request */
    if (num_profiles == 0) {
      std::cout << "\tRequesting " << _sickProfileFormatToString(profile_format) << " data stream from Sick LD..." << std::endl;
    } else {
      std::cout << "\tRequesting " << num_profiles << " " << _sickProfileFormatToString(profile_format) << " profiles from Sick LD..." << std::endl;
    }

    /* Request scan profiles from the Sick (empirically it can take the Sick up to a few seconds to respond) */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* Reset the payload buffer */
    memset(payload_buffer,0,6);

    /* Acquire the payload of the response */
    recv_message.GetPayload(payload_buffer);

    /* Check to make sure the returned format is correct and there were no errors */
    memcpy(&temp_buffer,&payload_buffer[2],2);
    temp_buffer = sick_ld_to_host_byte_order(temp_buffer);

    /* Another sanity check */
    if (temp_buffer != profile_format) {
      throw SickErrorException("SickLD::_getSickScanProfiles: Incorrect profile format was returned by the Sick LD!");
    }
  
    /* Check if the data stream flags need to be set */
    if (num_profiles == 0 && profile_format == SICK_SCAN_PROFILE_RANGE) {
      _sick_streaming_range_data = true;
    }
    else if (num_profiles == 0 && profile_format == SICK_SCAN_PROFILE_RANGE_AND_ECHO) {
      _sick_streaming_range_and_echo_data = true;
    }

    /* Show some output */
    if (num_profiles == 0) {
      std::cout << "\t\tData stream started!" << std::endl;
    } else {
      std::cout << "\t\tSick LD sending " << num_profiles << " scan profiles!" << std::endl;
    }
  
    /* Success */
  }

  /**
   * \brief Parses a well-formed sequence of bytes into a corresponding scan profile
   * \param *src_buffer The source data buffer
   * \param &profile_data The destination data structure
   */
  void SickLD::_parseScanProfile( uint8_t * const src_buffer, sick_ld_scan_profile_t &profile_data ) const {

    uint16_t profile_format = 0;
    unsigned int data_offset = 0;

    /* Extract the scan profile format from the buffer */
    memcpy(&profile_format,&src_buffer[data_offset],2);
    profile_format = sick_ld_to_host_byte_order(profile_format);
    data_offset += 2;

    /* Extract the number of sectors in the scan area */
    profile_data.num_sectors = src_buffer[data_offset+1];
    data_offset += 2;

    /* NOTE: For the following field definitions see page 32 of the
     *       Sick LD telegram listing.
     */
    uint16_t temp_buffer; // A temporary buffer
  
    /* Check if PROFILESENT is included */
    if (profile_format & 0x0001) {
      memcpy(&temp_buffer,&src_buffer[data_offset],2);
      profile_data.profile_number = sick_ld_to_host_byte_order(temp_buffer);
      data_offset += 2;
    }
  
    /* Check if PROFILECOUNT is included */
    if (profile_format & 0x0002) {
      memcpy(&temp_buffer,&src_buffer[data_offset],2);
      profile_data.profile_counter = sick_ld_to_host_byte_order(temp_buffer);
      data_offset += 2;
    }
  
    /* Check if LAYERNUM is included */
    if (profile_format & 0x0004) {
      memcpy(&temp_buffer,&src_buffer[data_offset],2);
      profile_data.layer_num = sick_ld_to_host_byte_order(temp_buffer);
      data_offset += 2;
    }
  
    /* The extraneous stuff is out of the way, now extract the data
     * for each of the sectors in the scan area...
     */
    for (unsigned int i=0; i < profile_data.num_sectors; i++) {

      /* Check if SECTORNUM is included */
      if (profile_format & 0x0008) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].sector_num = sick_ld_to_host_byte_order(temp_buffer);
	data_offset += 2;
      }
      else {
	profile_data.sector_data[i].sector_num = 0;
      }
    
      /* Check if DIRSTEP is included */
      if (profile_format & 0x0010) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].angle_step = ((double)sick_ld_to_host_byte_order(temp_buffer))/16;
	data_offset += 2;
      }
      else {
	profile_data.sector_data[i].angle_step = 0;
      }
    
      /* Check if POINTNUM is included */
      if (profile_format & 0x0020) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].num_data_points = sick_ld_to_host_byte_order(temp_buffer);
	data_offset += 2;
      }
      else {
	profile_data.sector_data[i].num_data_points = 0;
      }
    
      /* Check if TSTART is included */
      if (profile_format & 0x0040) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].timestamp_start = sick_ld_to_host_byte_order(temp_buffer);
	data_offset += 2;
      }
      else {
	profile_data.sector_data[i].timestamp_start = 0;
      }
      
      /* Check if STARTDIR is included */
      if (profile_format & 0x0080) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].angle_start = ((double)sick_ld_to_host_byte_order(temp_buffer))/16;
	data_offset += 2;
      }
      else {
	profile_data.sector_data[i].angle_start = 0;
      }
    
      /* Acquire the range and echo values for the sector */
      for (unsigned int j=0; j < profile_data.sector_data[i].num_data_points; j++) {

	/* Check if DISTANCE-n is included */
	if (profile_format & 0x0100) {
	  memcpy(&temp_buffer,&src_buffer[data_offset],2);
	  profile_data.sector_data[i].range_values[j] = ((double)sick_ld_to_host_byte_order(temp_buffer))/256;
	  data_offset += 2;
	}
	else {
	  profile_data.sector_data[i].range_values[j] = 0;
	}
      
	/* Check if DIRECTION-n is included */
	if (profile_format & 0x0200) {
	  memcpy(&temp_buffer,&src_buffer[data_offset],2);
	  profile_data.sector_data[i].scan_angles[j] = ((double)sick_ld_to_host_byte_order(temp_buffer))/16;
	  data_offset += 2;
	}
	else {
	  profile_data.sector_data[i].scan_angles[j] = 0;
	}
      
	/* Check if ECHO-n is included */
	if (profile_format & 0x0400) {
	  memcpy(&temp_buffer,&src_buffer[data_offset],2);
	  profile_data.sector_data[i].echo_values[j] = sick_ld_to_host_byte_order(temp_buffer);
	  data_offset += 2;      
	}
	else {
	  profile_data.sector_data[i].echo_values[j] = 0;
	}
	      
      }

      /* Check if TEND is included */
      if (profile_format & 0x0800) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].timestamp_stop = sick_ld_to_host_byte_order(temp_buffer);
	data_offset += 2;
      }
      else {
	profile_data.sector_data[i].timestamp_stop = 0;
      }
    
      /* Check if ENDDIR is included */
      if (profile_format & 0x1000) {
	memcpy(&temp_buffer,&src_buffer[data_offset],2);
	profile_data.sector_data[i].angle_stop = ((double)sick_ld_to_host_byte_order(temp_buffer))/16;
	data_offset += 2;   
      }
      else {
	profile_data.sector_data[i].angle_stop = 0;
      }
    
    }

    /* Check if SENSTAT is included */
    if (profile_format & 0x2000) {
      profile_data.sensor_status = src_buffer[data_offset+3] & 0x0F;
      profile_data.motor_status = (src_buffer[data_offset+3] >> 4) & 0x0F;
    }
    else {
      profile_data.sensor_status = SICK_SENSOR_MODE_UNKNOWN;
      profile_data.motor_status = SICK_MOTOR_MODE_UNKNOWN;
    }
  
  }

  /** 
   * \brief Kills the current data stream
   */
  void SickLD::_cancelSickScanProfiles( ) 
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device is in measurement mode */
    try {
      _setSickSensorModeToMeasure();
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_cancelSickScanProfiles: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_MEAS_SERV_CODE;           // Requested service type
    payload_buffer[1] = SICK_MEAS_SERV_CANCEL_PROFILE; // Requested service subtype
  
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,2);
    SickLDMessage recv_message;

    std::cout << "\tStopping the data stream..." << std::endl;

    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getFirmwareName: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Reset the payload buffer */
    memset(payload_buffer,0,2);
  
    /* Update the status of ths Sick LD */
    recv_message.GetPayload(payload_buffer);

    /* Extract and assign the sensor and motor status */
    _sick_sensor_mode = payload_buffer[5] & 0x0F;
    _sick_motor_mode = (payload_buffer[5] >> 4) & 0x0F;

    /* Since we just updated them, let's make sure everything s'ok */
    if (_sick_sensor_mode == SICK_SENSOR_MODE_ERROR) {
      throw SickErrorException("SickLD::_cancelSickScanProfiles: Sick LD returned sensor mode ERROR!");
    }

    /* Check the motor mode */
    if (_sick_motor_mode == SICK_MOTOR_MODE_ERROR) {
      throw SickErrorException("SickLD::_cancelSickScanProfiles: Sick LD returned motor mode ERROR!");
    }

    /* Set the stream flag for the driver */
    if (_sick_streaming_range_data) {
      _sick_streaming_range_data = false;
    }
    else {
      _sick_streaming_range_and_echo_data = false;
    }
  
    std::cout << "\t\tStream stopped!" << std::endl;    
  }  

  /** 
   * \brief Enables/disables nearfield suppression on the Sick LD
   * \param suppress_code Code indicating whether to enable or diable the nearfield suppression
   */
  void SickLD::_setSickFilter( const uint8_t suppress_code ) 
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device is not measuring */
    if (_sick_sensor_mode == SICK_SENSOR_MODE_MEASURE) {

      /* Set the Sick LD to rotate mode */
      try {
        _setSickSensorModeToRotate();
      }
         
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickFilter: Unknown exception!!!" << std::endl;
	throw;
      }  

    }
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;                 // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_SET_FILTER;           // Requested service subtype
    payload_buffer[3] = SICK_CONF_SERV_SET_FILTER_NEARFIELD; // Setting nearfield suppression filter    
    payload_buffer[5] = suppress_code;                       // Code telling whether to turn it on or off
  
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,6);
    SickLDMessage recv_message;

    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickFilter: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,6);
  
    /* Acquire the returned payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract FILTERITEM */
    uint16_t filter_item;
    memcpy(&filter_item,&payload_buffer[2],2);
    filter_item = sick_ld_to_host_byte_order(filter_item);

    /* Check that the returned filter item matches nearfiled suppression */
    if (filter_item != SICK_CONF_SERV_SET_FILTER_NEARFIELD) {
      throw SickErrorException("SickLD::_setSickFilter: Unexpected filter item returned from Sick LD!");
    }
  
    /* Success */
  }

  /**
   * \brief Get the parameters that define the Sick LD's identity
   */
  void SickLD::_getSickIdentity( )  throw( SickTimeoutException, SickIOException ) {

    try {
      
      _getSensorPartNumber();
      _getSensorName();
      _getSensorVersion();
      _getSensorSerialNumber();
      _getSensorEDMSerialNumber();
      _getFirmwarePartNumber();
      _getFirmwareName();
      _getFirmwareVersion();
      _getApplicationSoftwarePartNumber();
      _getApplicationSoftwareName();
      _getApplicationSoftwareVersion();

    }
 
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSickIdentity: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Success! */
  }

  /**
   * \brief Get the status of the Sick LD
   */
  void SickLD::_getSickStatus( ) throw( SickTimeoutException, SickIOException ) {

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_STAT_SERV_CODE;       // Requested service type
    payload_buffer[1] = SICK_STAT_SERV_GET_STATUS; // Requested service subtype
  
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,2);
    SickLDMessage recv_message;
  
    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
    
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickLD::_getSickStatus - Unknown exception!" << std::endl;
      throw;
    }
    
    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,2);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract the Sick LD's current sensor mode */
    _sick_sensor_mode = payload_buffer[5] & 0x0F;

    /* Extract the Sick LD's current motor mode */
    _sick_motor_mode = (payload_buffer[5] >> 4) & 0x0F;

    /* Success */
  }

  /**
   * \brief Sets the Sick LD's global parameters (sensor id, motor speed, and angular step) in flash.
   * \param sick_sensor_id The sensor id to be assigned to the unit.
   * \param sick_motor_speed The speed of the motor in Hz (must be in [5,20])
   * \param sick_angular_step The difference between two laser pulses in 1/16 degrees (e.g. sick_angular_step = 4 => 0.25 deg step).
   *                          Also, this value must evenly divide into 5760 and be greater than 1 (see telegram listing page 21).
   *
   * ALERT: This method writes the parameters to the Sick LD's flash, so there is no
   *        need to use it except when configuring the device.
   *
   * ALERT: This method DOES NOT DO ERROR CHECKING on the given parameter values.  It is the responsibility
   *        of the caller function to do error checking beforehand.
   */
  void SickLD::_setSickGlobalConfig( const uint8_t sick_sensor_id, const uint8_t sick_motor_speed, const double sick_angle_step )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device is in IDLE mode */
    try {
      _setSickSensorModeToIdle();
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickGlobalConfig: Unknown exception!!!" << std::endl;
      throw;
    }  

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;                      // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_SET_CONFIGURATION;         // Requested service subtype

    /* Set the configuration key */
    payload_buffer[3] = SICK_CONF_KEY_GLOBAL;                     // Use the global configuration key

    /* Set the message parameters */
    payload_buffer[5] = sick_sensor_id;                           // Include the given sensor ID
    payload_buffer[7] = sick_motor_speed;                         // Include the new Sick Motor speed value

    /* Set the angular step */
    uint16_t temp_buffer = _angleToTicks(sick_angle_step);
    temp_buffer = host_to_sick_ld_byte_order(temp_buffer);
    memcpy(&payload_buffer[8],&temp_buffer,2);

    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,10);
    SickLDMessage recv_message;

    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
              
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickGlobalConfig: Unknown exception!!!" << std::endl;
      throw;
    }  

    /* Reset the payload buffer */
    memset(payload_buffer,0,10);

    /* Extract the response payload */
    recv_message.GetPayload(payload_buffer);

    /* Check to make sure there wasn't an error */
    if (payload_buffer[2] != 0 || payload_buffer[3] != 0) {
      throw SickErrorException("SickLD::_setSickGlobalConfig: Configuration setting was NOT sucessful!");
    }

    /* Update the device driver with the new values */
    _sick_global_config.sick_sensor_id = sick_sensor_id;
    _sick_global_config.sick_motor_speed = sick_motor_speed;
    _sick_global_config.sick_angle_step = sick_angle_step;  
    
    /* Success! */
  }

  /**
   * \brief Get the global configuration of the Sick LD.
   */
  void SickLD::_getSickGlobalConfig( )  throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device is in IDLE mode */
    try {
      _setSickSensorModeToIdle();
    }
       
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSickGlobalConfig: Unknown exception!!!" << std::endl;
      throw;
    }  
  
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;              // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_GET_CONFIGURATION; // Requested service subtype
    payload_buffer[3] = SICK_CONF_KEY_GLOBAL;             // Configuration key
  
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;
  
    /* Send the message and check the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
            
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSickGlobalConfig: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,4);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract the configuration key */
    uint16_t temp_buffer = 0;
    unsigned int data_offset = 2;
    memcpy(&temp_buffer,&payload_buffer[data_offset],2);
    temp_buffer = sick_ld_to_host_byte_order(temp_buffer);
    data_offset += 2;

    /* A quick sanity check */
    if (temp_buffer != SICK_CONF_KEY_GLOBAL) {
      throw SickErrorException("SickLD::_getSickGlobalConfig: Unexpected message contents!");
    }

    /* Extract the global sensor ID */
    memcpy(&_sick_global_config.sick_sensor_id,&payload_buffer[data_offset],2);
    _sick_global_config.sick_sensor_id = sick_ld_to_host_byte_order(_sick_global_config.sick_sensor_id);
    data_offset += 2;
  
    /* Extract the nominal motor speed */
    memcpy(&_sick_global_config.sick_motor_speed,&payload_buffer[data_offset],2);
    _sick_global_config.sick_motor_speed = sick_ld_to_host_byte_order(_sick_global_config.sick_motor_speed);
    data_offset += 2;

    /* Extract the angular step */
    memcpy(&temp_buffer,&payload_buffer[data_offset],2);
    _sick_global_config.sick_angle_step = _ticksToAngle(sick_ld_to_host_byte_order(temp_buffer));
  
    /* Success */
  }

  /**
   * \brief Get the Sick LD's Ethernet configuration.
   */
  void SickLD::_getSickEthernetConfig( )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Ensure the device is in IDLE mode */
    try {      
      _setSickSensorModeToIdle();
    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    /* Set the service IDs */
    payload_buffer[0] = SICK_CONF_SERV_CODE;              // Requested service type
    payload_buffer[1] = SICK_CONF_SERV_GET_CONFIGURATION; // Requested service subtype
    payload_buffer[3] = SICK_CONF_KEY_ETHERNET;           // Configuration key
    
    /* Create the Sick messages */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;
    
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,4);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Extract the configuration key */
    uint16_t temp_buffer = 0;
    unsigned int data_offset = 2;
    memcpy(&temp_buffer,&payload_buffer[data_offset],2);
    temp_buffer = sick_ld_to_host_byte_order(temp_buffer);
    data_offset += 2;

    /* A quick sanity check */
    if (temp_buffer != SICK_CONF_KEY_ETHERNET) {
      throw SickErrorException("SickLD::_getSickEthernetConfig: Unexpected message contents!");
    }
  
    /* Extract the IP address of the Sick LD */
    for(unsigned int i=0; i < 4; i++,data_offset+=2) {
      memcpy(&_sick_ethernet_config.sick_ip_address[i],&payload_buffer[data_offset],2);
      _sick_ethernet_config.sick_ip_address[i] = sick_ld_to_host_byte_order(_sick_ethernet_config.sick_ip_address[i]);
    }

    /* Extract the associated subnet mask */
    for(unsigned int i=0; i < 4; i++,data_offset+=2) {
      memcpy(&_sick_ethernet_config.sick_subnet_mask[i],&payload_buffer[data_offset],2);
      _sick_ethernet_config.sick_subnet_mask[i] = sick_ld_to_host_byte_order(_sick_ethernet_config.sick_subnet_mask[i]);
    }

    /* Extract the default gateway */
    for(unsigned int i=0; i < 4; i++,data_offset+=2) {
      memcpy(&_sick_ethernet_config.sick_gateway_ip_address[i],&payload_buffer[data_offset],2);
      _sick_ethernet_config.sick_gateway_ip_address[i] = sick_ld_to_host_byte_order(_sick_ethernet_config.sick_gateway_ip_address[i]);
    }

    /* Extract the sick node ID (NOTE: This value doesn't matter, but we buffer it anyways) */
    memcpy(&_sick_ethernet_config.sick_node_id,&payload_buffer[data_offset],2);
    _sick_ethernet_config.sick_node_id = sick_ld_to_host_byte_order(_sick_ethernet_config.sick_node_id);
    data_offset += 2;

    /* Extract the transparent TCP port (NOTE: The significance of this value is unclear as
     * it doesn't affect the actual TCP port number that the Sick server is operating at.
     * But, we buffer it anyways as it is included in the configuration.)
     */
    memcpy(&_sick_ethernet_config.sick_transparent_tcp_port,&payload_buffer[data_offset],2);
    _sick_ethernet_config.sick_transparent_tcp_port = sick_ld_to_host_byte_order(_sick_ethernet_config.sick_transparent_tcp_port);
    data_offset += 2;

    /* Success */
  }

  /**
   * \brief Query the Sick for its current sector configuration.
   *
   * NOTE: Here we only buffer the sector stop angle as this is the
   *       only value returned by the GET_FUNCTION command. Using the
   *       stop angle does not give enough information to extract the
   *       start angle of sectors that aren't "normal measurement".
   */
  void SickLD::_getSickSectorConfig( )
    throw( SickErrorException, SickTimeoutException, SickIOException ) {

    /* Reset the sector config struct */
    memset(&_sick_sector_config,0,sizeof(sick_ld_config_sector_t));
    
    /* Get the configuration for all initialized sectors */
    for (unsigned int i = 0; i < SICK_MAX_NUM_SECTORS; i++) {
      
      /* Query the Sick for the function of the ith sector */
      try {
	_getSickSectorFunction(i,_sick_sector_config.sick_sector_functions[i],_sick_sector_config.sick_sector_stop_angles[i]);
      }
      
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_getSickSectorConfig: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Check if the sector is initialized */
      if (_sick_sector_config.sick_sector_functions[i] != SICK_CONF_SECTOR_NOT_INITIALIZED) {
	
	/* Check whether the sector is active (i.e. measuring) */
	if (_sick_sector_config.sick_sector_functions[i] == SICK_CONF_SECTOR_NORMAL_MEASUREMENT) {
	  _sick_sector_config.sick_active_sector_ids[_sick_sector_config.sick_num_active_sectors] = i;
	  _sick_sector_config.sick_num_active_sectors++;
	}
	
	/* Update the number of initialized sectors */
	_sick_sector_config.sick_num_initialized_sectors++;
      }    
      else {
	
	/* An uninitialized sector marks the end of the sector configuration */
	break;
      }
      
    } 
  
    /* Compute the starting angle for each of the initialized sectors */  
    for (unsigned int i = 1; i < _sick_sector_config.sick_num_initialized_sectors; i++) {
      _sick_sector_config.sick_sector_start_angles[i] = fmod(_sick_sector_config.sick_sector_stop_angles[i-1]+_sick_global_config.sick_angle_step,360);      
    }

    /* Determine the starting angle for the first sector */
    if (_sick_sector_config.sick_num_initialized_sectors > 1) {
      _sick_sector_config.sick_sector_start_angles[0] =
	fmod(_sick_sector_config.sick_sector_stop_angles[_sick_sector_config.sick_num_initialized_sectors-1]+_sick_global_config.sick_angle_step,360);
    }
  
    /* Success! */
  }

  /**
   * \brief Query the Sick LD for a particular ID string.
   * \param id_request_code The code indicating what ID string is being requested.
   * \param &id_return_string A reference to hold the string returned from the Sick LD.x
   */
  void SickLD::_getIdentificationString( const uint8_t id_request_code, std::string &id_return_string )
    throw( SickTimeoutException, SickIOException ) {
    
    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_STAT_SERV_CODE;   // Requested service type
    payload_buffer[1] = SICK_STAT_SERV_GET_ID; // Requested service subtype                  
    payload_buffer[3] = id_request_code;       // ID information that is being requested
  
    /* Create the Sick LD messages */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;

    /* Send the message and get the reply */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getIdentificationString: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer (not necessary, but it doesn't hurt to be careful) */
    memset(payload_buffer,0,4);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Assign the string */
    id_return_string = (char *) &payload_buffer[2];

    /* Success, woohooo! */
  }

  /**
   * \brief Get the Sick LD's part number
   */
  void SickLD::_getSensorPartNumber( )  throw( SickTimeoutException, SickIOException ) {

    /* Query the Sick LD */
    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_SENSOR_PART_NUM,_sick_identity.sick_part_number);
    }
          
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSensorPartNumber: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Success, woohooo! */
  }

  /**
   * \brief Get the Sick LD's assigned sensor name
   */
  void SickLD::_getSensorName( )  throw( SickTimeoutException, SickIOException ) {

    /* Query the Sick LD */
    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_SENSOR_NAME,_sick_identity.sick_name);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSensorName: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Get the Sick LD's sensor version
   */
  void SickLD::_getSensorVersion( ) throw( SickTimeoutException, SickIOException ) {

    /* Get the id info */
    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_SENSOR_VERSION,_sick_identity.sick_version);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSensorVersion: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Get the Sick LD's serial number
   */
  void SickLD::_getSensorSerialNumber( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_SENSOR_SERIAL_NUM,_sick_identity.sick_serial_number);
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSensorSerialNumber: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Get sensor EDM serial number
   */
  void SickLD::_getSensorEDMSerialNumber( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_SENSOR_EDM_SERIAL_NUM,_sick_identity.sick_edm_serial_number);
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getSensorEDMSerialNumber: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Ok */
  }

  /**
   * \brief Get firmware part number
   */
  void SickLD::_getFirmwarePartNumber( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_FIRMWARE_PART_NUM,_sick_identity.sick_firmware_part_number);
    }  
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getFirmwarePartNumber: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Get firmware name
   */
  void SickLD::_getFirmwareName( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_FIRMWARE_NAME,_sick_identity.sick_firmware_name);
    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getFirmwareName: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /** 
   * \brief Get firmware version number
   */
  void SickLD::_getFirmwareVersion( ) throw( SickTimeoutException, SickIOException ){

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_FIRMWARE_VERSION,_sick_identity.sick_firmware_version);
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getFirmwareVersion: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Ok */
  }

  /**
   * \brief Get application software part number
   */
  void SickLD::_getApplicationSoftwarePartNumber( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_APP_PART_NUM,_sick_identity.sick_application_software_part_number);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getApplicationSoftwarePartNumber: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Get application software name
   */
  void SickLD::_getApplicationSoftwareName( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_APP_NAME,_sick_identity.sick_application_software_name);
    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getApplication Software Name: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Get application software part number
   */
  void SickLD::_getApplicationSoftwareVersion( )  throw( SickTimeoutException, SickIOException ) {

    try {
      _getIdentificationString(SICK_STAT_SERV_GET_ID_APP_VERSION,_sick_identity.sick_application_software_version);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_getApplicationSoftwareVersion: Unknown exception!!!" << std::endl;
      throw;
    }
  
    /* Ok */
  }

  /**
   * \brief Attempts to set the "permanent" (by writing flash) operating values for the device
   * \param sick_motor_speed Desired motor speed for the device (hz)
   * \param sick_angle_step Desired scan angular resolution (deg)
   * \param active_sector_start_angles Angles marking the beginning of each desired active sector/area
   * \param active_sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_active_sectors The number of active sectors
   */
  void SickLD::_setSickGlobalParamsAndScanAreas( const unsigned int sick_motor_speed, const double sick_angle_step,
						 const double * const active_sector_start_angles,
						 const double * const active_sector_stop_angles,
						 const unsigned int num_active_sectors )
    throw( SickTimeoutException, SickIOException, SickConfigException, SickErrorException ) {

    /* Define buffers to hold the device-ready configuration */
    unsigned int num_sectors = 0;
    unsigned int sector_functions[SICK_MAX_NUM_SECTORS] = {0};
    double sector_stop_angles[SICK_MAX_NUM_SECTORS] = {0};
  
    /* A few dummy buffers */
    double sorted_active_sector_start_angles[SICK_MAX_NUM_SECTORS] = {0};
    double sorted_active_sector_stop_angles[SICK_MAX_NUM_SECTORS] = {0};
  
    /* Begin by checking the num of active sectors */
    if (num_active_sectors > SICK_MAX_NUM_SECTORS/2) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid number of active scan sectors!");
    }

    /* Ensure the given motor speed is valid (within proper bounds, etc...) */
    if (!_validSickMotorSpeed(sick_motor_speed)) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid motor speed!");
    }

    /* Ensure the scan resolution is valid (within proper bounds, etc...) */
    if (!_validSickScanResolution(sick_angle_step,active_sector_start_angles,active_sector_stop_angles,num_active_sectors)) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid scan resolution!");
    }

    /* Copy the input arguments */
    memcpy(sorted_active_sector_start_angles,active_sector_start_angles,sizeof(sorted_active_sector_start_angles));
    memcpy(sorted_active_sector_stop_angles,active_sector_stop_angles,sizeof(sorted_active_sector_stop_angles));

    /* Ensure a proper ordering of the given sector angle sets */
    _sortScanAreas(sorted_active_sector_start_angles,sorted_active_sector_stop_angles,num_active_sectors);
    
    /* Check for an invalid configuration */
    if (!_validActiveSectors(sorted_active_sector_start_angles,sorted_active_sector_stop_angles,num_active_sectors)) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid sector configuration!");
    }
  
    /* Ensure the resulting pulse frequency is valid for the device */
    if (!_validPulseFrequency(sick_motor_speed,sick_angle_step,sorted_active_sector_start_angles, sorted_active_sector_stop_angles,num_active_sectors)) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid pulse frequency!");
    }

    /* Generate the corresponding device-ready sector config */
    _generateSickSectorConfig(sorted_active_sector_start_angles,sorted_active_sector_stop_angles,num_active_sectors,sick_angle_step,
			      sector_functions,sector_stop_angles,num_sectors);

    try {
  
      /* Set the new sector configuration */
      _setSickSectorConfig(sector_functions,sector_stop_angles,num_sectors,false);

      /* Assign the new configuration in the flash
       *
       * NOTE: The following function must be called, even if the global parameters (motor speed,
       *       and angular resolution) are the same, in order to write the sector configuration.
       *       Why this is the case isn't exactly clear as the manual does not explain.
       */
      _setSickGlobalConfig(GetSickSensorID(),sick_motor_speed,sick_angle_step);

    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickGlobalParamsAndScanAreas: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    /* Success! */
  }

  /**
   * \brief Attempts to set the "temporary" (until a device reset) scan area config for the device
   * \param active_sector_start_angles Angles marking the beginning of each desired active sector/area
   * \param active_sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_active_sectors The number of active sectors
   */
  void SickLD::_setSickTemporaryScanAreas( const double * const active_sector_start_angles,
					   const double * const active_sector_stop_angles,
					   const unsigned int num_active_sectors )   throw( SickTimeoutException, SickIOException, SickConfigException ) {

    /* Define buffers to hold the device-ready configuration */
    unsigned int num_sectors = 0;
    unsigned int sector_functions[SICK_MAX_NUM_SECTORS] = {0};
    double sector_stop_angles[SICK_MAX_NUM_SECTORS] = {0};
  
    /* A few dummy buffers */
    double sorted_active_sector_start_angles[SICK_MAX_NUM_SECTORS] = {0};
    double sorted_active_sector_stop_angles[SICK_MAX_NUM_SECTORS] = {0};
  
    /* Begin by checking the num of active sectors */
    if (num_active_sectors > SICK_MAX_NUM_SECTORS/2)
      throw SickConfigException("_setSickTemporaryScanAreas: Invalid number of active scan sectors!");

    /* Copy the input arguments */
    memcpy(sorted_active_sector_start_angles,active_sector_start_angles,sizeof(sorted_active_sector_start_angles));
    memcpy(sorted_active_sector_stop_angles,active_sector_stop_angles,sizeof(sorted_active_sector_stop_angles));

    /* Ensure a proper ordering of the given sector angle sets */
    _sortScanAreas(sorted_active_sector_start_angles,sorted_active_sector_stop_angles,num_active_sectors);

    /* Check for an invalid configuration */
    if (!_validActiveSectors(sorted_active_sector_start_angles,sorted_active_sector_stop_angles,num_active_sectors)) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid sector configuration!");
    }
    
    /* Ensure the resulting pulse frequency is valid for the device */
    if (!_validPulseFrequency(GetSickMotorSpeed(),GetSickScanResolution(),sorted_active_sector_start_angles,
			      sorted_active_sector_stop_angles,num_active_sectors)) {
      throw SickConfigException("SickLD::_setSickGlobalParamsAndScanAreas: Invalid pulse frequency!");
    }
    
    /* Generate the corresponding device-ready sector config */
    _generateSickSectorConfig(sorted_active_sector_start_angles,sorted_active_sector_stop_angles,num_active_sectors,GetSickScanResolution(),
			      sector_functions,sector_stop_angles,num_sectors);

    /* Set the new sector configuration */
    try {
      _setSickSectorConfig(sector_functions,sector_stop_angles,num_sectors);
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickTemporaryScanAreas: Unknown exception!!!" << std::endl;
      throw;
    }  
    
    /* Success! */ 
  }

  /**
   * \brief Sets the sector configuration for the device
   * \param sector_functions Angles marking the beginning of each desired active sector/area
   * \param sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_sectors The total number of sectors in the configuration
   * \param set_flash_flag Indicates whether to mark the sectors for writing to flash w/ the next SET_CONFIG (global)
   */
  void SickLD::_setSickSectorConfig( const unsigned int * const sector_functions, const double * const sector_stop_angles,
				     const unsigned int num_sectors, const bool write_to_flash ) 
				     throw( SickErrorException, SickTimeoutException, SickIOException, SickConfigException ) {

    /* Assign the new sector configuration to the device */
    for (unsigned int sector_id = 0; sector_id < num_sectors; sector_id++) {

      try {
	
        /* Set the corresponding sector function */
        _setSickSectorFunction(sector_id,sector_functions[sector_id],sector_stop_angles[sector_id],write_to_flash);
    
        /* Resync the driver with the new sector configuration */
        _getSickSectorConfig();

      }
      
      /* Handle a timeout! */
      catch (SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Handle I/O exceptions */
      catch (SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickErrorException &sick_error_exception) {
	std::cerr << sick_error_exception.what() << std::endl;
	throw;
      }
      
      /* Handle a returned error code */
      catch (SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* A safety net */
      catch (...) {
	std::cerr << "SickLMS::_setSickSectorConfig: Unknown exception!!!" << std::endl;
	throw;
      }  

    }
  
    /* Success */
  }

  /**
   * \brief Sets the Sick LEDs and switching outputs 
   * \param sick_signal_flags Flags indicating which LEDs and SWITCHES to activate
   *
   * NOTE: This method does not preserve the previous state of the Sick's signals.
   *       In other words, only the signals flagged in sick_signal_flags will be
   *       set - all others will be off!
   */
  void SickLD::_setSickSignals( const uint8_t sick_signal_flags )  throw( SickIOException, SickTimeoutException, SickErrorException ) {

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLDMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the service IDs */
    payload_buffer[0] = SICK_STAT_SERV_CODE;       // Requested service type
    payload_buffer[1] = SICK_STAT_SERV_SET_SIGNAL; // Requested service subtype
    payload_buffer[3] = sick_signal_flags;         // PORTVAL
  
    /* Create the Sick message */
    SickLDMessage send_message(payload_buffer,4);
    SickLDMessage recv_message;
  
    /* Send the message and get a response */
    try {
      _sendMessageAndGetReply(send_message,recv_message);
    }
           
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    memset(payload_buffer,0,4);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Check to see if there was an error */
    if (payload_buffer[2] != 0) {
      throw SickErrorException("SickLD::_setSickSignals: Command failed!");
    }
  
    /* Success */
  }

  /**
   * \brief Send a message to the Sick LD and get its reply.
   * \param &send_message A reference to the well-formed message object that is to be sent.
   * \param &recv_message The destination message object for the received reply.
   * \param timeout_value The maximum time to allow for a response in seconds
   *
   * NOTE: This method also verifies the correct reply to the given send_message
   *       object is received.  So, if it fails, it may be due to an unexpected reply.
   */
  void SickLD::_sendMessageAndGetReply( const SickLDMessage &send_message, 
                                        SickLDMessage &recv_message, 
                                        const unsigned int timeout_value ) throw( SickIOException, SickTimeoutException ) {

    uint8_t byte_sequence[2] = {0};

    byte_sequence[0] = send_message.GetServiceCode() | 0x80;
    byte_sequence[1] = send_message.GetServiceSubcode();

    /* Send message and get reply using parent's method */
    try {
      SickLIDAR< SickLDBufferMonitor, SickLDMessage >::_sendMessageAndGetReply(send_message,recv_message,byte_sequence,2,0,DEFAULT_SICK_MESSAGE_TIMEOUT,1);
    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLD::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Flushes TCP receive buffer contents
   */
  void SickLD::_flushTCPRecvBuffer( ) throw( SickIOException, SickThreadException ) {

    uint8_t null_byte;
    int num_bytes_waiting = 0;    

    try {
    
      /* Acquire access to the data stream */
      _sick_buffer_monitor->AcquireDataStream();
      
      /* Acquire the number of the bytes awaiting read */
      if (ioctl(_sick_fd,FIONREAD,&num_bytes_waiting)) {
	throw SickIOException("SickLD::_flushTCPRecvBuffer: ioctl() failed! (Couldn't get the number of bytes awaiting read!)");
      }
      
      /* Read off the bytes awaiting in the buffer */
      for (int i = 0; i < num_bytes_waiting; i++) {
      	read(_sick_fd,&null_byte,1);
      }
      
      /* Release the stream */
      _sick_buffer_monitor->ReleaseDataStream();

    }

    /* Catch any serious IO exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* A sanity check */
    catch(...) {
      std::cerr << "SickLMS::_flushTerminalBuffer: Unknown exception!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Teardown TCP connection to Sick LD
   */
  void SickLD::_teardownConnection( ) throw( SickIOException ) {

    /* Close the socket! */
    if (close(_sick_fd) < 0) {
      throw SickIOException("SickLD::_teardownConnection: close() failed!");
    }
  
  }

  /**
   * \brief Generates a device-ready sector set given only an active sector spec.
   * \param *active_sector_start_angles Start angles for the active (measuring) sectors
   * \param *active_sector_stop_angles Stop angles for the active sectors
   * \param num_active_sectors The number of active sectors given
   * \param sick_angle_step The step angle (angular resolution) to be used in generating the config
   * \param *sector_functions An output buffer to hold the function assigned to each sector
   * \param *sector_stop_angles An output buffer to hold the stop angles associated w/ the generated sector set
   * \param &num_sectors The number of sectors in the final device-ready configuration
   */
  void SickLD::_generateSickSectorConfig( const double * const active_sector_start_angles,
					  const double * const active_sector_stop_angles,
					  const unsigned int num_active_sectors,
					  const double sick_angle_step,
					  unsigned int * const sector_functions,
					  double * const sector_stop_angles,
					  unsigned int &num_sectors ) const {

    num_sectors = 0;

    /* Generate the sector configuration for multiple sectors */
    double final_diff = 0;
    if (num_active_sectors > 1) {
    
      /* Generate the actual sector configuration for the device */
      for (unsigned int i = 0; i < num_active_sectors; i++) {

	/* Insert the measurement sector for the active area */
	sector_functions[num_sectors] = SICK_CONF_SECTOR_NORMAL_MEASUREMENT;
	sector_stop_angles[num_sectors] = active_sector_stop_angles[i];
	num_sectors++;	
      
	/* Check whether to insert a non-measurement sector */
	if ((i < num_active_sectors - 1) && (active_sector_start_angles[i+1]-active_sector_stop_angles[i] >= 2*sick_angle_step)) {

	  /* Set the next sector function as non-measurement */
	  sector_functions[num_sectors] = SICK_CONF_SECTOR_NO_MEASUREMENT;
	  sector_stop_angles[num_sectors] = active_sector_start_angles[i+1] - sick_angle_step;
	  num_sectors++;		

	}
      
      }

      /* Compute the difference between the final stop angle and the first start angle*/
      if (active_sector_stop_angles[num_active_sectors-1] < active_sector_start_angles[0]) {
	final_diff = active_sector_start_angles[0] - active_sector_stop_angles[num_active_sectors-1];
      }
      else {
	final_diff = active_sector_start_angles[0] + (360 - active_sector_stop_angles[num_active_sectors-1]);
      }        
    
    }
    else {
    
      /* Insert the measurement sector for the active area */
      sector_functions[num_sectors] = SICK_CONF_SECTOR_NORMAL_MEASUREMENT;
      sector_stop_angles[num_sectors] = active_sector_stop_angles[0];
      num_sectors++;	
    
      /* Compute the difference between the final stop angle and the first start angle*/
      if (active_sector_stop_angles[0] <= active_sector_start_angles[0]) {
	final_diff = active_sector_start_angles[0] - active_sector_stop_angles[num_active_sectors-1];
      }
      else {
	final_diff = active_sector_start_angles[0] + (360 - active_sector_stop_angles[num_active_sectors-1]);
      }        
    
    }

    /* Check whether to add a final non-measurement sector */
    if (final_diff >= 2*sick_angle_step) {

      /* Include the final non-measurement sector */
      sector_functions[num_sectors] = SICK_CONF_SECTOR_NO_MEASUREMENT;
      sector_stop_angles[num_sectors] = active_sector_start_angles[0]-sick_angle_step +
	                                360*(sick_angle_step > active_sector_start_angles[0]);
      num_sectors++;
    
    }    

    /* If necessary insert the non-initialized sector */
    if (num_sectors < SICK_MAX_NUM_SECTORS) {

      /* Include the uninitialized sector */
      sector_functions[num_sectors] = SICK_CONF_SECTOR_NOT_INITIALIZED;
      sector_stop_angles[num_sectors] = 0;
      num_sectors++;	    

    }

  }

  /**
   * \brief Converts encoder ticks to equivalent angle representation.
   * \param ticks The tick value to be converted
   * \return The corresponding angle value
   */
  double SickLD::_ticksToAngle( const uint16_t ticks ) const {
    return (((double)ticks)/16);
  }

  /**
   * \brief Converts encoder ticks to equivalent angle representation.
   * \param ticks The tick value to be converted
   * \return The corresponding ticks value
   *
   * NOTE: This function assumes that the angle value it receives is
   *       a multiple of 1/16 degree (which is the resolution of the
   *       encoder)
   */
  uint16_t SickLD::_angleToTicks( const double angle ) const {
    return (uint16_t)(angle*16);
  }

  /**
   * \brief Compute the mean pulse frequency (see page 22 of the operator's manual)
   * \param active_scan_area The total area where the Sick is actively scanning (in deg) (i.e. total area where the laser isn't blanked)
   * \param curr_motor_speed The current motor speed (in Hz)
   * \param curr_angular_resolution The current angular resolution of the Sick (in deg)
   * \return The mean pulse frequency for the given configuration parameters
   */
  double SickLD::_computeMeanPulseFrequency( const double active_scan_area, const double curr_motor_speed,
					     const double curr_angular_resolution ) const {
    /* Compute the mean pulse frequency */
    return _computeMaxPulseFrequency(SICK_MAX_SCAN_AREA,curr_motor_speed,curr_angular_resolution)*(active_scan_area/((double)SICK_MAX_SCAN_AREA));
  }

  /**
   * \brief Compute the mean pulse frequency (see page 22 of the operator's manual)
   * \param active_scan_area The total scan area that can be covered by the Sick
   * \param curr_motor_speed The current motor speed (in Hz)
   * \param curr_angular_resolution The current angular resolution of the Sick (in deg)
   * \return The maximum pulse frequency for the given configuration parameters
   */
  double SickLD::_computeMaxPulseFrequency( const double total_scan_area, const double curr_motor_speed,
					    const double curr_angular_resolution ) const {
    /* Compute the maximum pulse frequency */
    return total_scan_area*curr_motor_speed*(1/curr_angular_resolution);
  }

  /**
   * \brief Checks whether the given senor id is valid for the device.
   * \param sick_sensor_id Sick sensor ID
   */
  bool SickLD::_validSickSensorID( const unsigned int sick_sensor_id ) const {
    
    /* Ensure the sensor ID is valid */
    if (sick_sensor_id < SICK_MIN_VALID_SENSOR_ID || sick_sensor_id > SICK_MAX_VALID_SENSOR_ID) {
      return false;
    }  
    
    /* Success */
    return true;
  }

  /**
   * \brief Checks whether the given sick motor speed is valid for the device.
   * \param sick_motor_speed The sick motor speed (Hz)
   */ 
  bool SickLD::_validSickMotorSpeed( const unsigned int sick_motor_speed ) const {

    /* Check the validity of the new Sick LD motor speed */
    if (sick_motor_speed < SICK_MIN_MOTOR_SPEED || sick_motor_speed > SICK_MAX_MOTOR_SPEED) {
      return false;
    }

    /* Success */
    return true;
  }

  /**
   * \brief Checks whether the given scan resolution is valid
   * \param sick_scan_resolution Scan resolution of the device
   * \param sector_start_angles An array of the sector start angles
   * \param sector_stop_angles An array of the sector stop angles
   */ 
  bool SickLD::_validSickScanResolution( const double sick_angle_step, const double * const sector_start_angles,
					 const double * const sector_stop_angles, const unsigned int num_sectors ) const {

    /* Check the validity of the new Sick LD angular step */
    if (sick_angle_step < SICK_MAX_SCAN_ANGULAR_RESOLUTION || fmod(sick_angle_step,SICK_MAX_SCAN_ANGULAR_RESOLUTION) != 0) {
      std::cerr << "Invalid scan resolution! (should be a positive multiple of " << SICK_MAX_SCAN_ANGULAR_RESOLUTION << ")" << std::endl;
      return false;
    }
  
    /* Ensure that the sector boundaries are divisible by the desired step angle */
    for(unsigned int i = 0; i < num_sectors; i++) {
      
      /* Check both the sector start and stop angles */
      if (fmod(sector_start_angles[i],sick_angle_step) != 0 || fmod(sector_stop_angles[i],sick_angle_step) != 0) {
	std::cerr << "Invalid scan resolution! (sector boundaries must be evenly divisible by the step angle)" << std::endl;
	return false;
      }
    
    }

    /* Success */
    return true;
  }

  /**
   * \brief Checks whether the given configuration yields a valid mean and max pulse frequency (uses current sector config)
   * \param sick_motor_speed Desired sick motor speed (Hz)
   * \param sick_angle_step Desired scan angular resolution (deg)
   */ 
  bool SickLD::_validPulseFrequency( const unsigned int sick_motor_speed, const double sick_angle_step ) const {

    /* Simply call the other function w/ the current sector config and the given motor and step angle values */
    if(!_validPulseFrequency(sick_motor_speed,sick_angle_step,_sick_sector_config.sick_sector_start_angles,
			     _sick_sector_config.sick_sector_stop_angles,_sick_sector_config.sick_num_active_sectors)) {
      return false;
    }
    
    /* Valid! */
    return true;
  }

  /**
   * \brief Checks whether the given configuration yields a valid mean and max pulse frequency (uses given sector config)
   * \param sick_motor_speed Desired sick motor speed (Hz)
   * \param sick_angle_step Desired scan angular resolution (deg)
   * \param active_sector_start_angles Angles marking the beginning of each desired active sector/area
   * \param active_sector_stop_angles Angles marking the end of each desired active sector/area
   * \param num_active_sectors The number of active sectors/scan areas are given
   */ 
  bool SickLD::_validPulseFrequency( const unsigned int sick_motor_speed, const double sick_angle_step,
				     const double * const active_sector_start_angles,
				     const double * const active_sector_stop_angles,
				     const unsigned int num_active_sectors ) const {
  
    /* Compute the scan area */
    double scan_area = _computeScanArea(sick_angle_step,active_sector_start_angles,active_sector_stop_angles,num_active_sectors);
  
    /* Check the mean pulse rate of the desired configuration */
    if (_computeMeanPulseFrequency(scan_area,sick_motor_speed,sick_angle_step) > SICK_MAX_MEAN_PULSE_FREQUENCY) { 
      std::cerr << "Max mean pulse frequency exceeded! (try a slower motor speed, a larger step angle and/or a smaller active scan area)" << std::endl;
      return false;
    }

    /* Check the maximum pulse rate of the desired configuration */
    if (_computeMaxPulseFrequency(SICK_MAX_SCAN_AREA,sick_motor_speed,sick_angle_step) > SICK_MAX_PULSE_FREQUENCY) { 
      std::cerr << "Max pulse frequency exceeded! (try a slower motor speed, a larger step angle and/or a smaller active scan area)" << std::endl;
      return false;
    }
  
    /* Valid! */
    return true;
  }

  /**
   * \brief Computes the active scan area for the Sick given the current
   *        sector configuration
   * \param sick_angle_step The angular resolution of the Sick LD
   * \param active_sector_start_angles The start angles for the active scan sectors
   * \param active_sector_stop_angles The stop angles for the active scan sectors
   * \param num_active_sectors The number of active sectors
   * \return The Sick LD scan area corresponding to the given bounds 
   *
   * NOTE: The Sick LD computes scan area by subtracting the end of the
   *       previous sector from the end of the current sector.  As such, we have 
   *       to add a single step angle to our computation in order to get the scan
   *       area that is used by the Sick LD.  Unfortunately, this is how the device
   *       does its computation (as opposed to using the angle at which the first
   *       scan in the given sector is taken) so this is how we do it.
   *       
   */
  double SickLD::_computeScanArea( const double sick_angle_step, const double * const active_sector_start_angles,
				   const double * const active_sector_stop_angles, const unsigned int num_active_sectors ) const {

    /* Define the current scan area */
    double total_scan_area = 0;
    double curr_sector_scan_area = 0;
  
    /* For each sector given sum the absolute scan area for it */
    for (unsigned int i = 0; i < num_active_sectors; i++) {

      /* Compute the total scan area for this sector */
      curr_sector_scan_area = fabs(active_sector_start_angles[i]-active_sector_stop_angles[i]);

      /* Update the total scan area */
      total_scan_area += curr_sector_scan_area + sick_angle_step;
    }
  
    /* Return the computed area */
    return total_scan_area;  

  }

  /**
   * \brief Sort the scan areas based on the given angles to place them in device "scan" order
   * \param sector_start_angles Array of angles (deg) defining the starting position of each active sector
   * \param sector_stop_angles Array of angles (deg) defining the stopping position of each active sector
   * \param num_active_sectors Number of active sectors 
   */
  void SickLD::_sortScanAreas( double * const sector_start_angles, double * const sector_stop_angles,
			       const unsigned int num_sectors ) const {

    /* A dummy temp variable */
    double temp = 0;
  
    /* Employ a simple bubblesort (NOTE: Only at most a handful of values will have to be sorted) */
    for (unsigned int i = 0; i < num_sectors; i++) {
      for (unsigned int j = (num_sectors-1); j > i; j--) {
	if (sector_start_angles[j] < sector_start_angles[j-1]) {
	  SWAP_VALUES(sector_start_angles[j],sector_start_angles[j-1],temp);
	  SWAP_VALUES(sector_stop_angles[j],sector_stop_angles[j-1],temp);
	}
      }
    }
  
  }

  /**
   * \brief Determines wheter a given set of sector bounds are valid.
   * \param sector_start_angles Array of angles (deg) defining the starting position of each active sector
   * \param sector_stop_angles Array of angles (deg) defining the stopping position of each active sector
   * \param num_active_sectors Number of active sectors
   */
  bool SickLD::_validActiveSectors( const double * const sector_start_angles, const double * const sector_stop_angles,
				    const unsigned int num_sectors ) const {

    /* A sanity check to make sure all are in [0,360) */
    for (unsigned int i = 0; i < num_sectors; i++) {

      if (sector_start_angles[i] < 0 || sector_stop_angles[i] < 0 ||
	  sector_start_angles[i] >= 360 || sector_stop_angles[i] >= 360) {

	std::cerr << "Invalid sector config! (all degree values must be in [0,360))" << std::endl;
	return false;
      }
    
    }
  
    /* If multiple sectors are defined */
    if (num_sectors > 1) {

      /* Check whether the given sector arrangement is overlapping */
      for (unsigned int i = 0; i < (num_sectors - 1); i++) {
	if (sector_start_angles[i] > sector_stop_angles[i] || sector_stop_angles[i] >= sector_start_angles[i+1]) {
	  std::cerr << "Invalid sector definitions! (check sector bounds)" << std::endl;
	  return false;
	}    
      }
    
      /* Check the last sector against the first */    
      if (sector_stop_angles[num_sectors-1] <= sector_start_angles[num_sectors-1] &&
	  sector_stop_angles[num_sectors-1] >= sector_start_angles[0]) {
	std::cerr << "Invalid sector definitions! (check sector bounds)" << std::endl;
	return false;
      }
    
    }

    /* Valid! */
    return true;
  }

  /**
   * \brief Check that the given profile format is supported by the current driver version.
   * \param profile_format The requested profile format.
   * \return True if the given scan profile is supported by the driver
   */
  bool SickLD::_supportedScanProfileFormat( const uint16_t profile_format ) const {

    /* Check the supplied scan profile format */
    switch(profile_format) {
    case SICK_SCAN_PROFILE_RANGE:
      return true;
    case SICK_SCAN_PROFILE_RANGE_AND_ECHO:
      return true;
    default:
      return false;
    }
  }

  /**
   * \brief Print data corresponding to the referenced sector data structure.
   * \param &sector_data The sector configuration to be printed
   */
  void SickLD::_printSectorProfileData( const sick_ld_sector_data_t &sector_data ) const {
  
    std::cout << "\t---- Sector Data " << sector_data.sector_num << " ----" << std::endl;
    std::cout << "\tSector Num.: " <<  sector_data.sector_num << std::endl;
    std::cout << "\tSector Angle Step (deg): " << sector_data.angle_step<< std::endl;
    std::cout << "\tSector Num. Data Points: " << sector_data.num_data_points << std::endl;
    std::cout << "\tSector Start Timestamp (ms): " << sector_data.timestamp_start << std::endl;
    std::cout << "\tSector Stop Timestamp (ms): " << sector_data.timestamp_stop << std::endl;
    std::cout << "\tSector Start Angle (deg): " << sector_data.angle_start << std::endl;
    std::cout << "\tSector Stop Angle (deg): " << sector_data.angle_stop << std::endl;
    std::cout << std::flush;
  }

  /**
   * \brief Prints data fields of the given scan profile.
   * \param profile_data The scan profile to be printed.
   * \param print_sector_data Indicates whether to print the sector data fields associated
   *                          with the given profile.
   */
  void SickLD::_printSickScanProfile( const sick_ld_scan_profile_t profile_data, const bool print_sector_data ) const {
  
    std::cout << "\t========= Sick Scan Prof. =========" << std::endl;
    std::cout << "\tProfile Num.: " << profile_data.profile_number << std::endl;
    std::cout << "\tProfile Counter: " << profile_data.profile_counter << std::endl;
    std::cout << "\tLayer Num.: " << profile_data.layer_num << std::endl;
    std::cout << "\tSensor Status: " <<  _sickSensorModeToString(profile_data.sensor_status) << std::endl;
    std::cout << "\tMotor Status: " <<  _sickMotorModeToString(profile_data.motor_status) << std::endl;
    std::cout << "\tNum. Sectors: " << profile_data.num_sectors << std::endl;

    // Print the corresponding active and non-active sector data
    for (unsigned int i=0; i < profile_data.num_sectors && print_sector_data; i++) {
      _printSectorProfileData(profile_data.sector_data[i]);
    }
    
    std::cout << "\t====================================" << std::endl;
    std::cout << std::flush;
  }

  /**
   * \brief Map Sick LD sensor modes to their equivalent service subcode representations
   * \param sick_sensor_mode The sensor mode to be converted
   * \return The corresponding work service subcode
   */
  uint8_t SickLD::_sickSensorModeToWorkServiceSubcode( const uint8_t sick_sensor_mode ) const {

    switch(sick_sensor_mode) {
    case SICK_SENSOR_MODE_IDLE:
      return SICK_WORK_SERV_TRANS_IDLE;
    case SICK_SENSOR_MODE_ROTATE:
      return SICK_WORK_SERV_TRANS_ROTATE;
    case SICK_SENSOR_MODE_MEASURE:
      return SICK_WORK_SERV_TRANS_MEASURE;
    default:
      std::cerr << "SickLD::_sickSensorModeToWorkServiceSubcode: Invalid sensor mode! (Returning 0)" << std::endl;
      return 0; //Something is seriously wrong if we end up here!
    }
  }

  /**
   * \brief Converts the Sick LD numerical sensor mode to a representative string
   * \param sick_sensor_mode The sensor mode to be converted
   * \return The corresponding sensor_mode string
   */
  std::string SickLD::_sickSensorModeToString( const uint8_t sick_sensor_mode ) const {

    switch(sick_sensor_mode) {
    case SICK_SENSOR_MODE_IDLE:
      return "IDLE";
    case SICK_SENSOR_MODE_ROTATE:
      return "ROTATE (laser is off)";
    case SICK_SENSOR_MODE_MEASURE:
      return "MEASURE (laser is on)";
    case SICK_SENSOR_MODE_ERROR:
      return "ERROR";
    case SICK_SENSOR_MODE_UNKNOWN:
      return "UNKNOWN";
    default:
      return "UNRECOGNIZED!!!";
    }
  }

  /**
   * \brief Converts the Sick LD numerical motor mode to a representative string
   * \param sick_motor_mode The sick motor mode code to be converted
   * \return The corresponding string
   */
  std::string SickLD::_sickMotorModeToString( const uint8_t sick_motor_mode ) const {

    switch(sick_motor_mode) {
    case SICK_MOTOR_MODE_OK:
      return "OK";
    case SICK_MOTOR_MODE_SPIN_TOO_HIGH:
      return "SPIN TOO HIGH";
    case SICK_MOTOR_MODE_SPIN_TOO_LOW:
      return "SPIN TOO LOW";
    case SICK_MOTOR_MODE_ERROR:
      return "ERROR";
    case SICK_MOTOR_MODE_UNKNOWN:
      return "UNKNOWN";
    default:
      return "UNRECOGNIZED!!!";
    }
  }

  /**
   * \brief Converts return value from TRANS_MEASURE to a representative string
   * \param return_value The TRANS_MEASURE numeric return code
   * \return The corresponding string
   */
  std::string SickLD::_sickTransMeasureReturnToString( const uint8_t return_value ) const {

    switch(return_value) {
    case SICK_WORK_SERV_TRANS_MEASURE_RET_OK:
      return "LD-OEM/LD-LRS Measures";
    case SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_MAX_PULSE:
      return "Max Pulse Frequency Too High";
    case SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_MEAN_PULSE:
      return "Mean Pulse Frequency Too High";
    case SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_SECT_BORDER:
      return "Sector Borders Not Configured Correctly";
    case SICK_WORK_SERV_TRANS_MEASURE_RET_ERR_SECT_BORDER_MULT:
      return "Sector Borders Not Multiple of Angle Step";
    default:
      return "UNRECOGNIZED!!!";
    }
  
  }

  /**
   * \brief Converts the Sick LD numerical reset level to a representative string
   * \param The numeric reset level code
   * \return The corresponding string
   */
  std::string SickLD::_sickResetLevelToString( const uint16_t reset_level ) const {

    switch(reset_level) {
    case SICK_WORK_SERV_RESET_INIT_CPU:
      return "RESET (CPU Reinitialized)";
    case SICK_WORK_SERV_RESET_KEEP_CPU:
      return "RESET (CPU Not Reinitialized)";
    case SICK_WORK_SERV_RESET_HALT_APP:
      return "RESET (Halt App. and Enter IDLE)";
    default:
      return "UNRECOGNIZED!!!";
    }
  
  }

  /**
   * \brief Converts the Sick LD numerical sector config to a representative string
   * \param sick_sector_function The numeric sector function to be converted
   * \return The corresponding string
   */
  std::string SickLD::_sickSectorFunctionToString( const uint16_t sick_sector_function ) const {

    switch(sick_sector_function) {
    case SICK_CONF_SECTOR_NOT_INITIALIZED:
      return "NOT INITIALIZED";
    case SICK_CONF_SECTOR_NO_MEASUREMENT:
      return "NOT MEASURING";
    case SICK_CONF_SECTOR_RESERVED:
      return "RESERVED";
    case SICK_CONF_SECTOR_NORMAL_MEASUREMENT:
      return "MEASURING";
    case SICK_CONF_SECTOR_REFERENCE_MEASUREMENT:
      return "REFERENCE";
    default:
      return "UNRECOGNIZED!!!";
    }
  
  }

  /**
   * \brief Converts the Sick LD numerical motor mode to a representative string
   * \param profile_format The profile format to be converted
   * \return The corresponding string
   */
  std::string SickLD::_sickProfileFormatToString( const uint16_t profile_format ) const {

    switch(profile_format) {
    case SICK_SCAN_PROFILE_RANGE:
      return "RANGE ONLY";
    case SICK_SCAN_PROFILE_RANGE_AND_ECHO:
      return "RANGE + ECHO";
    default:
      return "UNRECOGNIZED!!!";
    }
  
  }

  /**
   * \brief Prints the initialization footer.
   */
  void SickLD::_printInitFooter( ) const {

    std::cout << "\t*** Init. complete: Sick LD is online and ready!" << std::endl; 
    std::cout << "\tNum. Active Sectors: " << (int)_sick_sector_config.sick_num_active_sectors << std::endl;  
    std::cout << "\tMotor Speed: " << _sick_global_config.sick_motor_speed << " (Hz)" << std::endl;
    std::cout << "\tScan Resolution: " << _sick_global_config.sick_angle_step << " (deg)" << std::endl;
    std::cout << std::endl;
    
  }

} //namespace SickToolbox
