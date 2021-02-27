/*!
 * \file SickLMS2xx.cc
 * \brief Definition of the class SickLMS2xx
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
#include <sstream>
#include <iostream>
#include <termios.h>
#include <sys/ioctl.h>
#include <signal.h>

#include <sicktoolbox/SickLMS2xx.hh>
#include <sicktoolbox/SickLMS2xxMessage.hh>
#include <sicktoolbox/SickLMS2xxBufferMonitor.hh>
#include <sicktoolbox/SickLMS2xxUtility.hh>
#include <sicktoolbox/SickException.hh>

#ifdef HAVE_LINUX_SERIAL_H
#include <linux/serial.h>
#else
#define B500000 0010005
#endif

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief Primary constructor
   * \param sick_device_path The path of the device
   */
  SickLMS2xx::SickLMS2xx( const std::string sick_device_path ): SickLIDAR< SickLMS2xxBufferMonitor, SickLMS2xxMessage >( ),
								_sick_device_path(sick_device_path),
								_curr_session_baud(SICK_BAUD_UNKNOWN),
								_desired_session_baud(SICK_BAUD_UNKNOWN),
								_sick_type(SICK_LMS_TYPE_UNKNOWN),
								_sick_mean_value_sample_size(0),
								_sick_values_subrange_start_index(0),
								_sick_values_subrange_stop_index(0)
  {
    
    /* Initialize the protected/private structs */
    memset(&_sick_operating_status,0,sizeof(sick_lms_2xx_operating_status_t));
    memset(&_sick_software_status,0,sizeof(sick_lms_2xx_software_status_t));
    memset(&_sick_restart_status,0,sizeof(sick_lms_2xx_restart_status_t));
    memset(&_sick_pollution_status,0,sizeof(sick_lms_2xx_pollution_status_t));
    memset(&_sick_signal_status,0,sizeof(sick_lms_2xx_signal_status_t));
    memset(&_sick_field_status,0,sizeof(sick_lms_2xx_field_status_t));
    memset(&_sick_baud_status,0,sizeof(sick_lms_2xx_baud_status_t));
    memset(&_sick_device_config,0,sizeof(sick_lms_2xx_device_config_t));
    memset(&_old_term,0,sizeof(struct termios));
    
  }

  /**
   * \brief Destructor
   */
  SickLMS2xx::~SickLMS2xx() {

    try {

      /* Attempt to uninitialize the device */
      _teardownConnection();
      
    }

    /* Catch an I/O exception */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::~SickLMS2xx: Unknown exception!" << std::endl;
    }
    
  }
  
  /**
   * \brief Attempts to initialize the Sick LMS 2xx and then sets communication at
   *        at the given baud rate.
   * \param desired_baud_rate Desired session baud rate
   * \param delay Delay to wait for SICK to power on (in seconds)
   */
  void SickLMS2xx::Initialize( const sick_lms_2xx_baud_t desired_baud_rate, const uint32_t delay )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Buffer the desired baud rate in case we have to reset */
    _desired_session_baud = desired_baud_rate;
    
    try {
    
      std::cout << std::endl << "\t*** Attempting to initialize the Sick LMS..." << std::endl << std::flush;
      
      /* Initialize the serial term for communication */
      std::cout << "\tAttempting to open device @ " << _sick_device_path << std::endl << std::flush;
      _setupConnection(delay);
      std::cout << "\t\tDevice opened!" << std::endl << std::flush;

      /* Start/reset the buffer monitor */
      if (!_sick_monitor_running) {
	std::cout << "\tAttempting to start buffer monitor..." << std::endl;       
	_startListening();
	std::cout << "\t\tBuffer monitor started!" << std::endl;
      }
      else {
	std::cout << "\tAttempting to reset buffer monitor..." << std::endl;       
	_sick_buffer_monitor->SetDataStream(_sick_fd);
	std::cout << "\t\tBuffer monitor reset!" << std::endl;       
      }

      try {

	std::cout << "\tAttempting to set requested baud rate..." << std::endl;
	_setSessionBaud(_desired_session_baud);
	
      }

      /* Assume a timeout is due to a misconfigured terminal baud */
      catch(SickTimeoutException &sick_timeout) {
      
	/* Check whether to do an autodetect */
	sick_lms_2xx_baud_t default_baud = _baudToSickBaud(DEFAULT_SICK_LMS_2XX_SICK_BAUD);
	std::cout << "\tFailed to set requested baud rate..." << std::endl << std::flush;
	std::cout << "\tAttempting to detect LMS baud rate..." << std::endl << std::flush;
	if((default_baud != SICK_BAUD_9600) && _testSickBaud(SICK_BAUD_9600)) {
	  std::cout << "\t\tDetected LMS baud @ " << SickBaudToString(SICK_BAUD_9600) << "!" << std::endl;
	} else if((default_baud != SICK_BAUD_19200) && _testSickBaud(SICK_BAUD_19200)) {
	  std::cout << "\t\tDetected LMS baud @ " << SickBaudToString(SICK_BAUD_19200) << "!" << std::endl;
	} else if((default_baud != SICK_BAUD_38400) && _testSickBaud(SICK_BAUD_38400)) {
	  std::cout << "\t\tDetected LMS baud @ " << SickBaudToString(SICK_BAUD_38400) << "!" << std::endl;
	} else if((default_baud) != SICK_BAUD_500K && _testSickBaud(SICK_BAUD_500K)) {
	  std::cout << "\t\tDetected LMS baud @ " << SickBaudToString(SICK_BAUD_500K) << "!" << std::endl;
	} else {
          _stopListening();
	  throw SickIOException("SickLMS2xx::Initialize: failed to detect baud rate!");	
	}
	std::cout << std::flush;
	
	/* Try again! */
	if (_curr_session_baud != _desired_session_baud) {
	  std::cout << "\tAttempting to setup desired baud (again)..." << std::endl << std::flush;
	  _setSessionBaud(_desired_session_baud);	  
	}
	
      }

      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::Initialize: Unknown exception!" << std::endl;
	throw;
      }

      std::cout << "\t\tOperating @ " << SickBaudToString(_curr_session_baud) << std::endl;     
      
      /* Set the device to request range mode */
      _setSickOpModeMonitorRequestValues();
      
      /* Acquire the type of device that we are working with */
      std::cout << "\tAttempting to sync driver..." << std::endl << std::flush;
      _getSickType();     // Get the Sick device type string
      _getSickStatus();   // Get the Sick device status
      _getSickConfig();   // Get the Sick current config
      std::cout << "\t\tDriver synchronized!" << std::endl << std::flush;

      /* Set the flag */
      _sick_initialized = true;
      
    }

    /* Handle a config exception */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::Initialize: Unknown exception!" << std::endl;
      throw;
    }

    /* Initialization was successful! */
    std::cout << "\t*** Init. complete: Sick LMS is online and ready!" << std::endl; 
    std::cout << "\tSick Type: " << SickTypeToString(GetSickType()) << std::endl;
    std::cout << "\tScan Angle: " << GetSickScanAngle() << " (deg)" << std::endl;  
    std::cout << "\tScan Resolution: " << GetSickScanResolution() << " (deg)" << std::endl;
    std::cout << "\tMeasuring Mode: " << SickMeasuringModeToString(GetSickMeasuringMode()) << std::endl;
    std::cout << "\tMeasuring Units: " << SickMeasuringUnitsToString(GetSickMeasuringUnits()) << std::endl;
    std::cout << std::endl << std::flush;
    
  }

  /**
   * \brief Uninitializes the LMS by putting it in a mode where it stops streaming data,
   *        and returns it to the default baud rate (specified in the header).
   */
  void SickLMS2xx::Uninitialize( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    if (_sick_initialized) {

      std::cout << std::endl << "\t*** Attempting to uninitialize the Sick LMS..." << std::endl;       

      try {
	
	/* Restore original operating mode */
	_setSickOpModeMonitorRequestValues();
	
	/* Restore original baud rate settings */
	_setSessionBaud(_baudToSickBaud(DEFAULT_SICK_LMS_2XX_SICK_BAUD));

	/* Attempt to cancel the buffer monitor */
	if (_sick_monitor_running) {
	  std::cout << "\tAttempting to stop buffer monitor..." << std::endl;
	  _stopListening();
	  std::cout << "\t\tBuffer monitor stopped!" << std::endl;
	}
	
	std::cout << "\t*** Uninit. complete - Sick LMS is now offline!" << std::endl << std::flush;
	
      }
    
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << " (attempting to kill connection anyways)" << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::Unintialize: Unknown exception!!!" << std::endl;
	throw;
      }

      /* Reset the flag */
      _sick_initialized = false;
      
    }
      
  }

  /**
   * \brief Gets the Sick LMS 2xx device path
   * \return The device path as a std::string
   */
  std::string SickLMS2xx::GetSickDevicePath( ) const {
    return _sick_device_path;
  }
  
  /**
   * \brief Gets the Sick LMS 2xx type
   * \return The device type
   */
  sick_lms_2xx_type_t SickLMS2xx::GetSickType( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickType: Sick LMS is not initialized!");
    }

    /* Return the Sick LMS type */
    return _sick_type;
    
  }

  /**
   * \brief Gets the current scan angle of the device
   * \return Scan angle of the device (sick_lms_2xx_scan_angle_t) 
   */
  double SickLMS2xx::GetSickScanAngle( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickScanAngle: Sick LMS is not initialized!");
    }

    /* Return the Sick scan angle */
    return (double)_sick_operating_status.sick_scan_angle;

  }

  /**
   * \brief Gets the current angular resolution
   * \return Angular resolution of the Sick LMS 2xx (sick_lms_2xx_scan_resolution_t) 
   */
  double SickLMS2xx::GetSickScanResolution( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickScanResolution: Sick LMS is not initialized!");
    }

    /* Return the scan resolution */
    return _sick_operating_status.sick_scan_resolution*(0.01);

  }
    
  /**
   * \brief Sets the measurement units for the device
   * \param sick_units Desired measurement units for the device
   */
  void SickLMS2xx::SetSickMeasuringUnits( const sick_lms_2xx_measuring_units_t sick_units )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::SetSickMeasuringUnits: Sick LMS is not initialized!");
    }

    /* Ensure a valid units type was given */
    if (!_validSickMeasuringUnits(sick_units)) {
      throw SickConfigException("SickLMS2xx::SetSickMeasuringUnits: Undefined measurement units!");
    }

    /* Make sure the write is necessary */
    if (sick_units != _sick_device_config.sick_measuring_units) {
      
      /* Setup a local copy of the config */
      sick_lms_2xx_device_config_t sick_device_config;
      
      /* Copy the configuration locally */
      sick_device_config = _sick_device_config;

      /* Set the units value */
      sick_device_config.sick_measuring_units = sick_units;

      try {
	
	/* Attempt to set the new configuration */
      	_setSickConfig(sick_device_config);
	
      }
	
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
      	std::cerr << sick_config_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
      	std::cerr << sick_timeout_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
      	std::cerr << sick_io_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
      	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
      	std::cerr << "SickLMS2xx::SetSickMeasuringUnits: Unknown exception!!!" << std::endl;
      	throw;
      }

    }
    else {
      std::cerr << "\tSickLMS2xx::SetSickMeasuringUnits - Device is already configured w/ these units. (skipping write)" << std::endl;
    }
      
  }

  /**
   * \brief Gets the current Sick LMS 2xx measuring units
   * \return Measuring units (sick_lms_2xx_measuring_units_t)
   */
  sick_lms_2xx_measuring_units_t SickLMS2xx::GetSickMeasuringUnits( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickMeasuringUnits: Sick LMS is not initialized!");
    }

    /* Return the measurement units */
    return (sick_lms_2xx_measuring_units_t)_sick_operating_status.sick_measuring_units;

  }
  
  /**
   * \brief Sets the Sick LMS sensitivity level
   * \param sick_sensitivity Desired sensitivity level
   */
  void SickLMS2xx::SetSickSensitivity( const sick_lms_2xx_sensitivity_t sick_sensitivity )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::SetSickSensitivity: Sick LMS is not initialized!");
    }
    
    /* Ensure the command is supported by the given Sick model */
    if (!_isSickLMS211() && !_isSickLMS221() && !_isSickLMS291()) {
      throw SickConfigException("SickLMS2xx::SetSickSensitivity: This command is not supported by this Sick model!");
    }

    /* Ensure this is a valid sick sensitivity value*/
    if (!_validSickSensitivity(sick_sensitivity)) {
      throw SickConfigException("SickLMS2xx::SetSickSensitivity: Undefined sensitivity level!");
    }

    /* Make sure the write is necessary */
    if (sick_sensitivity != _sick_device_config.sick_peak_threshold) {
      
      /* Setup a local copy of the config */
      sick_lms_2xx_device_config_t sick_device_config;
      
      /* Copy the configuration locally */
      sick_device_config = _sick_device_config;

      /* Set the units value */
      sick_device_config.sick_peak_threshold = sick_sensitivity;

      try {
	
	/* Attempt to set the new configuration */
      	_setSickConfig(sick_device_config);
	
      }
	
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
      	std::cerr << sick_config_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
      	std::cerr << sick_timeout_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
      	std::cerr << sick_io_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
      	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
      	std::cerr << "SickLMS2xx::SetSickSensitivity: Unknown exception!!!" << std::endl;
      	throw;
      }

    }
    else {
      std::cerr << "\tSickLMS2xx::SetSickSensitivity - Sick is already operating at this sensitivity level! (skipping write)" << std::endl;
    }
      
  }

  /**
   * \brief Sets the Sick LMS sensitivity level
   * \param sick_sensitivity Desired sensitivity level
   */
  void SickLMS2xx::SetSickPeakThreshold( const sick_lms_2xx_peak_threshold_t sick_peak_threshold )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::SetSickPeakThreshold: Sick LMS is not initialized!");
    }
    
    /* Ensure the command is supported by the given Sick model */
    if (!_isSickLMS200() && !_isSickLMS220()) {
      throw SickConfigException("SickLMS2xx::SetSickPeakThreshold: This command is not supported by this Sick model!");
    }
    
    /* Ensure this is a valid sick sensitivity value*/
    if (!_validSickPeakThreshold(sick_peak_threshold)) {
      throw SickConfigException("SickLMS2xx::SetSickPeakThreshold: Undefined peak threshold!");
    }

    /* Make sure the write is necessary */
    if (sick_peak_threshold != _sick_device_config.sick_peak_threshold) {
      
      /* Setup a local copy of the config */
      sick_lms_2xx_device_config_t sick_device_config;
      
      /* Copy the configuration locally */
      sick_device_config = _sick_device_config;

      /* Set the units value */
      sick_device_config.sick_peak_threshold = sick_peak_threshold;

      try {
	
	/* Attempt to set the new configuration */
      	_setSickConfig(sick_device_config);
	
      }
	
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
      	std::cerr << sick_config_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
      	std::cerr << sick_timeout_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
      	std::cerr << sick_io_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
      	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
      	std::cerr << "SickLMS2xx::SetSickPeakThreshold: Unknown exception!!!" << std::endl;
      	throw;
      }

    }
    else {
      std::cerr << "\tSickLMS2xx::SetSickPeakThreshold - Sick is already operating w/ given threshold! (skipping write)" << std::endl;
    }
    
  }
  
  /**
   * \brief Gets the current Sick LMS 2xx sensitivity level
   * \return Sensitivity level (sick_lms_2xx_sensitivity_t)
   */
  sick_lms_2xx_sensitivity_t SickLMS2xx::GetSickSensitivity( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickSensitivity: Sick LMS is not initialized!");
    }
    
    /* Make sure sensitivity is something that is defined for this model */
    if(!_isSickLMS211() && !_isSickLMS221() && !_isSickLMS291()) {
      std::cerr << "Sensitivity is undefined for model: " << SickTypeToString(GetSickType()) << " (returning \"Unknown\")" << std::endl;
      return SICK_SENSITIVITY_UNKNOWN;
    }

    /* If its supported than return the actual value */
    return (sick_lms_2xx_sensitivity_t)_sick_device_config.sick_peak_threshold;  //If the device is 211/221/291 then this value is sensitivity

  }

  /**
   * \brief Gets the current Sick LMS 2xx sensitivity level
   * \return Sensitivity level (sick_lms_2xx_sensitivity_t)
   */
  sick_lms_2xx_peak_threshold_t SickLMS2xx::GetSickPeakThreshold( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickPeakThreshold: Sick LMS is not initialized!");
    }
    
    /* Make sure sensitivity is something that is defined for this model */
    if(!_isSickLMS200() && !_isSickLMS220()) {
      std::cerr << "Peak threshold is undefined for model: " << SickTypeToString(GetSickType()) << " (returning \"Unknown\")" << std::endl;
      return SICK_PEAK_THRESHOLD_UNKNOWN;
    }

    /* If its supported than return the actual value */
    return (sick_lms_2xx_peak_threshold_t)_sick_device_config.sick_peak_threshold;  //If the device is 211/221/291 then this value is sensitivity

  }
  
  /**
   * \brief Sets the measuring mode for the device
   * \param sick_measuring_mode Desired measuring mode
   */
  void SickLMS2xx::SetSickMeasuringMode( const sick_lms_2xx_measuring_mode_t sick_measuring_mode )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::SetSickMeasuringUnits: Sick LMS is not initialized!");
    }
    
    /* Ensure this is a valid sick sensitivity value*/
    if (!_validSickMeasuringMode(sick_measuring_mode)) {
      throw SickConfigException("SickLMS2xx::SetSickMeasuringMode: Undefined measuring mode!");
    }

    /* Make sure the write is necessary */
    if (sick_measuring_mode != _sick_device_config.sick_measuring_mode) {
      
      /* Setup a local copy of the config */
      sick_lms_2xx_device_config_t sick_device_config;
      
      /* Copy the configuration locally */
      sick_device_config = _sick_device_config;

      /* Set the units value */
      sick_device_config.sick_measuring_mode = sick_measuring_mode;

      try {
	
	/* Attempt to set the new configuration */
      	_setSickConfig(sick_device_config);
	
      }
	
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
      	std::cerr << sick_config_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
      	std::cerr << sick_timeout_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
      	std::cerr << sick_io_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
      	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
      	std::cerr << "SickLMS2xx::SetSickMeasuringMode: Unknown exception!!!" << std::endl;
      	throw;
      }

    }
    else {
      std::cerr << "\tSickLMS2xx::SetSickMeasuringMode - Sick is already operating w/ this measuring mode! (skipping write)" << std::endl;
    }
      
  }

  /**
   * \brief Gets the current Sick LMS 2xx measuring mode
   * \return Measuring mode (sick_lms_2xx_measuring_mode_t)
   */
  sick_lms_2xx_measuring_mode_t SickLMS2xx::GetSickMeasuringMode( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickMeasuringMode: Sick LMS is not initialized!");
    }

    /* Return the determined measuring mode */
    return (sick_lms_2xx_measuring_mode_t)_sick_operating_status.sick_measuring_mode;

  }

  /**
   * \brief Gets the current Sick LMS 2xx operating mode
   * \return Operating mode (sick_lms_2xx_operating_mode_t)
   */
  sick_lms_2xx_operating_mode_t SickLMS2xx::GetSickOperatingMode( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickScanAngle: Sick LMS is not initialized!");
    }

    /* Return the current operating mode of the device */
    return (sick_lms_2xx_operating_mode_t)_sick_operating_status.sick_operating_mode;

  }
  
  /**
   * \brief Sets the availability level of the device
   * \param sick_availability_level Desired availability of the Sick LMS
   */
  void SickLMS2xx::SetSickAvailability( const uint8_t sick_availability_flags )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::SetSickAvailabilityFlags: Sick LMS is not initialized!");
    }
    
    /* Ensure this is a valid sick sensitivity value*/
    if ( sick_availability_flags > 7 ) {      
      throw SickConfigException("SickLMS2xx::SetSickAvailabilityFlags: Invalid availability!");
    }

    /* Setup a local copy of the config */
    sick_lms_2xx_device_config_t sick_device_config;
    
    /* Copy the configuration locally */
    sick_device_config = _sick_device_config;
    
    /* Maintain the higher level bits */
    sick_device_config.sick_availability_level &= 0xF8;
    
    /* Set the new availability flags */
    sick_device_config.sick_availability_level |= sick_availability_flags;

        /* Make sure the write is necessary */
    if (sick_device_config.sick_availability_level != _sick_device_config.sick_availability_level) {
      
      try {
	
	/* Attempt to set the new configuration */
      	_setSickConfig(sick_device_config);
	
      }
	
      /* Handle any config exceptions */
      catch(SickConfigException &sick_config_exception) {
      	std::cerr << sick_config_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
      	std::cerr << sick_timeout_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
      	std::cerr << sick_io_exception.what() << std::endl;
      	throw;
      }
      
      /* Handle any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
      	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Handle anything else */
      catch(...) {
      	std::cerr << "SickLMS2xx::SetSickAvailabilityFlags: Unknown exception!!!" << std::endl;
      	throw;
      }

    }
    else {
      std::cerr << "\tSickLMS2xx::SetSickAvailability - Device is already operating w/ given availability. (skipping write)" << std::endl;
    }
    
  }

  /**
   * \brief Gets the current Sick LMS 2xx availability level flags
   * \return Sick LMS 2xx Availability level flags
   */
  uint8_t SickLMS2xx::GetSickAvailability( ) const throw( SickConfigException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickAvailabilityFlags: Sick LMS is not initialized!");
    }

    /* Return the availability flags */
    return _sick_device_config.sick_availability_level;

  }
  
  /*!
   * \brief Sets the variant of the Sick LMS 2xx (scan angle and scan resolution)
   * \param scan_angle The desired scan angle of the Sick LMS 2xx
   * \param scan_resolution The desired angular resolution of the Sick LMS 2xx
   */
  void SickLMS2xx::SetSickVariant( const sick_lms_2xx_scan_angle_t scan_angle, const sick_lms_2xx_scan_resolution_t scan_resolution )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::SetSickVariant: Sick LMS is not initialized!");
    }
    
    /* A sanity check to make sure that the command is supported */
    if (_sick_type == SICK_LMS_TYPE_211_S14 || _sick_type == SICK_LMS_TYPE_221_S14 ||_sick_type == SICK_LMS_TYPE_291_S14) {
      throw SickConfigException("SickLMS2xx::SetSickVariant: Command not supported on this model!");
    }

    /* Ensure the given scan angle is defined */
    if (!_validSickScanAngle(scan_angle)) {
      throw SickConfigException("SickLMS2xx::SetSickVariant: Undefined scan angle!");
    }

    /* Ensure the given scan resolution is defined */
    if (!_validSickScanResolution(scan_resolution)) {
      throw SickConfigException("SickLMS2xx::SetSickMeasuringUnits: Undefined scan resolution!");
    }

    SickLMS2xxMessage message, response;
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    payload_buffer[0] = 0x3B; // Command to set sick variant

    /* Map the scan angle */
    switch(scan_angle) {
    case SICK_SCAN_ANGLE_100:
      payload_buffer[1] = 0x64;
      break;
    case SICK_SCAN_ANGLE_180:
      payload_buffer[1] = 0xB4;
      break;
    default:
      throw SickConfigException("SickLMS2xx::SetSickVariant: Given scan angle is invalid!");
    }
    
    /* Map the resolution */
    switch(scan_resolution) {
    case SICK_SCAN_RESOLUTION_100:
      payload_buffer[3] = 0x64;
      break;
    case SICK_SCAN_RESOLUTION_50:
      payload_buffer[3] = 0x32;
      break;
    case SICK_SCAN_RESOLUTION_25:
      payload_buffer[3] = 0x19;
      break;
    default:
      throw SickConfigException("SickLMS2xx::SetSickVariant: Given scan resolution is invalid!");
    }
    
    /* Build the request message */
    message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,5);
    
    try {

      /*
       * Set the device to request measured mode
       *
       * NOTE: This is done since the Sick stops sending
       *       data if the variant is reset  midstream.
       */
      _setSickOpModeMonitorRequestValues();
      
      /* Send the variant request and get a reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);
      
    }

    /* Handle a config exception */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::SetSickVariant: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Extract the payload length */
    response.GetPayload(payload_buffer);

    /* Check if the configuration was successful */
    if(payload_buffer[1] != 0x01) {
      throw SickConfigException("SickLMS2xx::SetSickVariant: Configuration was unsuccessful!");
    }

    /* Update the scan angle of the device */
    memcpy(&_sick_operating_status.sick_scan_angle,&payload_buffer[2],2);
    _sick_operating_status.sick_scan_angle =
      sick_lms_2xx_to_host_byte_order(_sick_operating_status.sick_scan_angle);
    
    /* Update the angular resolution of the device */
    memcpy(&_sick_operating_status.sick_scan_resolution,&payload_buffer[4],2);
    _sick_operating_status.sick_scan_resolution =
      sick_lms_2xx_to_host_byte_order(_sick_operating_status.sick_scan_resolution);
    
  }

  /**
   * \brief Returns the most recent measured values obtained by the Sick LMS 2xx
   * \param *measurement_values Destination buffer for holding the current round of measured values
   * \param &num_measurement_values Number of values stored in measurement_values
   * \param *sick_field_a_values Stores the Field A values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_b_values Stores the Field B values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_c_values Stores the Field C values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)
   *
   * NOTE: Calling this function will return either range or reflectivity measurements
   *       depending upon the current measuring mode of the device.
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick LMS 2xx for this value to be populated.
   */
  void SickLMS2xx::GetSickScan( unsigned int * const measurement_values,
			     unsigned int & num_measurement_values,
			     unsigned int * const sick_field_a_values,
			     unsigned int * const sick_field_b_values,
			     unsigned int * const sick_field_c_values,
			     unsigned int * const sick_telegram_index,
			     unsigned int * const sick_real_time_scan_index ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickScan: Sick LMS is not initialized!");
    }
    
    /* Declare message objects */
    SickLMS2xxMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {
    
      /* Restore original operating mode */
      _setSickOpModeMonitorStreamValues();
      
      /* Receive a data frame from the stream. */
      _recvMessage(response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT);
      
      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xB0) {
	throw SickIOException("SickLMS2xx::GetSickScan: Unexpected message!");
      }

      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);

      /* Define a local scan profile object */
      sick_lms_2xx_scan_profile_b0_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_lms_2xx_scan_profile_b0_t));

      /* Parse the message payload */
      _parseSickScanProfileB0(&payload_buffer[1],sick_scan_profile);

      /* Return the request values! */
      num_measurement_values = sick_scan_profile.sick_num_measurements;

      for (unsigned int i = 0; i < num_measurement_values; i++) {

	/* Copy the measurement value */
	measurement_values[i] = sick_scan_profile.sick_measurements[i];

	/* If requested, copy field A values */
	if(sick_field_a_values) {
	  sick_field_a_values[i] = sick_scan_profile.sick_field_a_values[i];
	}

	/* If requested, copy field B values */
	if(sick_field_b_values) {
	  sick_field_b_values[i] = sick_scan_profile.sick_field_b_values[i];
	}

	/* If requested, copy field C values */
	if(sick_field_c_values) {
	  sick_field_c_values[i] = sick_scan_profile.sick_field_c_values[i];
	}

      }

      /* If requested, copy the real time scan index */
      if(sick_real_time_scan_index) {
	*sick_real_time_scan_index = sick_scan_profile.sick_real_time_scan_index;
      }
      
      /* If requested, copy the telegram index */
      if(sick_telegram_index) {
	*sick_telegram_index = sick_scan_profile.sick_telegram_index;
      }

    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetSickScan: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Acquires both range and reflectivity values from the Sick LMS 211/221/291-S14 (LMS-FAST)
   * \param *range_values The buffer in which range measurements will be stored
   * \param *reflect_values The buffer in which reflectivity measurements will be stored
   * \param reflect_subrange_start_index The starting index of the desired measured reflectivity subrange
   * \param relfect_subrange_stop_index The stopping index of the desired measured relfectivity subrange
   * \param &num_range_measurements The number of range measurements stored in range_values
   * \param &num_reflectivity_measurements The number of reflectivity measurements stored in reflect_values
   * \param *sick_field_a_values Stores the Field A values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_b_values Stores the Field B values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_c_values Stores the Field C values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick LMS 2xx for this value to be populated.
   */
  void SickLMS2xx::GetSickScan( unsigned int * const range_values,
			     unsigned int * const reflect_values,
			     unsigned int & num_range_measurements,
			     unsigned int & num_reflect_measurements,
			     unsigned int * const sick_field_a_values,
			     unsigned int * const sick_field_b_values,
			     unsigned int * const sick_field_c_values,
			     unsigned int * const sick_telegram_index,
			     unsigned int * const sick_real_time_scan_index ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickScan: Sick LMS is not initialized!");
    }
    
    /* Declare message objects */
    SickLMS2xxMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {
      
      /* Restore original operating mode */
      _setSickOpModeMonitorStreamRangeAndReflectivity();
      
      /* Receive a data frame from the stream. */
      _recvMessage(response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT);

      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xC4) {
	throw SickIOException("SickLMS2xx::GetSickScan: Unexpected message!");
      }
      
      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);
      
      /* Define a local scan profile object */
      sick_lms_2xx_scan_profile_c4_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_lms_2xx_scan_profile_c4_t));

      /* Parse the message payload */
      _parseSickScanProfileC4(&payload_buffer[1],sick_scan_profile);

      /* Return the requested values! */
      num_range_measurements = sick_scan_profile.sick_num_range_measurements;
      num_reflect_measurements = sick_scan_profile.sick_num_reflect_measurements;
      
      for (unsigned int i = 0; i < sick_scan_profile.sick_num_range_measurements; i++) {

	/* Copy the measurement value */
	range_values[i] = sick_scan_profile.sick_range_measurements[i];
	
	/* If requested, copy field A values */
	if(sick_field_a_values) {
	  sick_field_a_values[i] = sick_scan_profile.sick_field_a_values[i];
	}

	/* If requested, copy field B values */
	if(sick_field_b_values) {
	  sick_field_b_values[i] = sick_scan_profile.sick_field_b_values[i];
	}

	/* If requested, copy field C values */
	if(sick_field_c_values) {
	  sick_field_c_values[i] = sick_scan_profile.sick_field_c_values[i];
	}

      }

      /* Copy the reflectivity measurements */
      for( unsigned int i = 0; i < num_reflect_measurements; i++) {
	reflect_values[i] = sick_scan_profile.sick_reflect_measurements[i];
      }
      
      /* If requested, copy the telegram index */
      if(sick_telegram_index) {
	*sick_telegram_index = sick_scan_profile.sick_telegram_index;
      }
      
      /* If requested, copy the real time scan index */
      if(sick_real_time_scan_index) {
	*sick_real_time_scan_index = sick_scan_profile.sick_real_time_scan_index;
      }
      
    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetSickScan: Unknown exception!!!" << std::endl;
      throw;
    }

  }
  
  /**
   * \brief Returns the most recent measured values from the corresponding subrange
   * \param sick_subrange_start_index The starting index of the desired subrange (See below for example)
   * \param sick_subrange_stop_index The stopping index of the desired subrange (See below for example)
   * \param *measurement_values Destination buffer for holding the current round of measured value
   * \param &num_measurement_values Number of values stored in measurement_values
   * \param *sick_field_a_values Stores the Field A values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_b_values Stores the Field B values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_c_values Stores the Field C values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)*
   *
   * NOTE: Calling this function will return either range or reflectivity measurements
   *       depending upon the current measuring mode of the device.
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick LMS 2xx for this value to be populated.
   *
   * EXAMPLE: Using a 180/0.5 Sick LMS variant, a subrange over the scan area [0,10.5] would
   *          correspond to setting the index arguments as follows: sick_subrange_start = 1,
   *          sick_subrange_stop = 22
   */  
  void SickLMS2xx::GetSickScanSubrange( const uint16_t sick_subrange_start_index,
				     const uint16_t sick_subrange_stop_index,
				     unsigned int * const measurement_values,
				     unsigned int & num_measurement_values,
				     unsigned int * const sick_field_a_values,
				     unsigned int * const sick_field_b_values,
				     unsigned int * const sick_field_c_values,
				     unsigned int * const sick_telegram_index,
				     unsigned int * const sick_real_time_scan_index ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickScanSubrange: Sick LMS is not initialized!");
    }
    
    /* Declare message object */
    SickLMS2xxMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {
    
      /* Restore original operating mode */
      _setSickOpModeMonitorStreamValuesSubrange(sick_subrange_start_index,sick_subrange_stop_index);
      
      /* Receive a data frame from the stream. */
      _recvMessage(response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT);
      
      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xB7) {
	throw SickIOException("SickLMS2xx::GetSickScanSubrange: Unexpected message!");
      }

      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);

      /* Define a local scan profile object */
      sick_lms_2xx_scan_profile_b7_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_lms_2xx_scan_profile_b7_t));

      /* Parse the message payload */
      _parseSickScanProfileB7(&payload_buffer[1],sick_scan_profile);

      /* Return the request values! */
      num_measurement_values = sick_scan_profile.sick_num_measurements;

      for (unsigned int i = 0; i < num_measurement_values; i++) {

	/* Copy the measurement value */
	measurement_values[i] = sick_scan_profile.sick_measurements[i];

	/* If requested, copy field A values */
	if(sick_field_a_values) {
	  sick_field_a_values[i] = sick_scan_profile.sick_field_a_values[i];
	}

	/* If requested, copy field B values */
	if(sick_field_b_values) {
	  sick_field_b_values[i] = sick_scan_profile.sick_field_b_values[i];
	}

	/* If requested, copy field C values */
	if(sick_field_c_values) {
	  sick_field_c_values[i] = sick_scan_profile.sick_field_c_values[i];
	}

      }

      /* If requested, copy the real time scan index */
      if(sick_real_time_scan_index) {
	*sick_real_time_scan_index = sick_scan_profile.sick_real_time_scan_index;
      }
      
      /* If requested, copy the telegram index */
      if(sick_telegram_index) {
	*sick_telegram_index = sick_scan_profile.sick_telegram_index;
      }

    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetSickScanSubrange: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Returns the most recent partial scan obtained by Sick LMS 2xx
   * \param *measurement_values Destination buffer for holding the current round of measured value
   * \param &num_measurement_values Number of values stored in measurement_values
   * \param *sick_partial_scan_index Partial scan index associated w/ returned values (0 = 0.00 deg, 1 = 0.25 deg, 2 = 0.50 deg, 3 = 0.75 deg)
   * \param *sick_field_a_values Stores the Field A values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_b_values Stores the Field B values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_field_c_values Stores the Field C values associated with the given scan (Default: NULL => Not wanted)
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)
   *
   * NOTE: This function will set the device to interlaced mode.
   *
   * NOTE: The current angular resolution defines the minimum step size in the partial
   *       scan sequence.  To obtain a high resolution partial scan you must set the
   *       Sick LMS 2xx variant to use a 0.25 degrees scan resolution.
   * 
   * NOTE: Calling this function will return either range or reflectivity measurements
   *       depending upon the current measuring mode of the device.
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick LMS 2xx for this value to be populated.
   */  
  void SickLMS2xx::GetSickPartialScan( unsigned int * const measurement_values,
				    unsigned int & num_measurement_values,
				    unsigned int & partial_scan_index,
				    unsigned int * const sick_field_a_values,
				    unsigned int * const sick_field_b_values,
				    unsigned int * const sick_field_c_values,
				    unsigned int * const sick_telegram_index,
				    unsigned int * const sick_real_time_scan_index ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickPartialScan: Sick LMS is not initialized!");
    }
    
    /* Declare message objects */
    SickLMS2xxMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {

      /* Restore original operating mode */
      _setSickOpModeMonitorStreamValuesFromPartialScan();
      
      /* Receive a data frame from the stream. */
      _recvMessage(response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT);
      
      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xB0) {
	throw SickIOException("SickLMS2xx::GetSickPartialScan: Unexpected message!");
      }

      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);

      /* Define a local scan profile object */
      sick_lms_2xx_scan_profile_b0_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_lms_2xx_scan_profile_b0_t));

      /* Parse the message payload */
      _parseSickScanProfileB0(&payload_buffer[1],sick_scan_profile);

      /* Return the request values! */
      num_measurement_values = sick_scan_profile.sick_num_measurements;

      /* Assign the partial scan index */
      partial_scan_index = sick_scan_profile.sick_partial_scan_index;
      
      for (unsigned int i = 0; i < num_measurement_values; i++) {

	/* Copy the measurement value */
	measurement_values[i] = sick_scan_profile.sick_measurements[i];

	/* If requested, copy field A values */
	if(sick_field_a_values) {
	  sick_field_a_values[i] = sick_scan_profile.sick_field_a_values[i];
	}

	/* If requested, copy field B values */
	if(sick_field_b_values) {
	  sick_field_b_values[i] = sick_scan_profile.sick_field_b_values[i];
	}

	/* If requested, copy field C values */
	if(sick_field_c_values) {
	  sick_field_c_values[i] = sick_scan_profile.sick_field_c_values[i];
	}

      }

      /* If requested, copy the real time scan index */
      if(sick_real_time_scan_index) {
	*sick_real_time_scan_index = sick_scan_profile.sick_real_time_scan_index;
      }
      
      /* If requested, copy the telegram index */
      if(sick_telegram_index) {
	*sick_telegram_index = sick_scan_profile.sick_telegram_index;
      }

    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetSickPartialScan: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Returns the most recent mean measured values from the Sick LMS 2xx.
   * \param sick_sample_size Number of scans to consider in computing the mean measured values (NOTE: 2 <= sick_sample_size <= 250)
   * \param *measurement_values Destination buffer for holding the current round of measured value
   * \param &num_measurement_values Number of values stored in measurement_values
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)
   *
   * NOTE: Calling this function will return either range or reflectivity measurements
   *       depending upon the current measuring mode of the device.
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick LMS 2xx for this value to be populated.
   */  
  void SickLMS2xx::GetSickMeanValues( const uint8_t sick_sample_size,
				   unsigned int * const measurement_values,
				   unsigned int & num_measurement_values,
				   unsigned int * const sick_telegram_index,
				   unsigned int * const sick_real_time_scan_index ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickMeanValues: Sick LMS is not initialized!");
    }
    
    /* Declare message objects */
    SickLMS2xxMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {

      /* Restore original operating mode */
      _setSickOpModeMonitorStreamMeanValues(sick_sample_size);
      
      /* Receive a data frame from the stream. (NOTE: Can take 10+ seconds for a reply) */
      _recvMessage(response,DEFAULT_SICK_LMS_2XX_SICK_MEAN_VALUES_MESSAGE_TIMEOUT);
      
      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xB6) {
	throw SickIOException("SickLMS2xx::GetSickMeanValues: Unexpected message!");
      }

      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);

      /* Define a local scan profile object */
      sick_lms_2xx_scan_profile_b6_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_lms_2xx_scan_profile_b6_t));

      /* Parse the message payload */
      _parseSickScanProfileB6(&payload_buffer[1],sick_scan_profile);

      /* Return the request values! */
      num_measurement_values = sick_scan_profile.sick_num_measurements;

      for (unsigned int i = 0; i < num_measurement_values; i++) {
	/* Copy the measurement value */
	measurement_values[i] = sick_scan_profile.sick_measurements[i];
      }

      /* If requested, copy the real time scan index */
      if(sick_real_time_scan_index) {
	*sick_real_time_scan_index = sick_scan_profile.sick_real_time_scan_index;
      }
      
      /* If requested, copy the telegram index */
      if(sick_telegram_index) {
	*sick_telegram_index = sick_scan_profile.sick_telegram_index;
      }

    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetSickMeanValues: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Returns the most recent mean measured values from the specified subrange
   * \param sick_sample_size Number of consecutive scans used to compute the mean values
   * \param sick_subrange_start_index The starting index of the desired subrange (See below for example)
   * \param sick_subrange_stop_index The stopping index of the desired subrange (See below for example)
   * \param *measurement_values Destination buffer for holding the current round of measured value
   * \param &num_measurement_values Number of values stored in measurement_values
   * \param *sick_telegram_index The telegram index assigned to the message (modulo: 256) (Default: NULL => Not wanted)
   * \param *sick_real_time_scan_index The real time scan index for the latest message (module 256) (Default: NULL => Not wanted)
   *
   * NOTE: Calling this function will return either range or reflectivity measurements
   *       depending upon the current measuring mode of the device.
   *
   * NOTE: Real-time scan indices must be enabled by setting the corresponding availability
   *       of the Sick LMS 2xx for this value to be populated.
   *
   * EXAMPLE: Using a 180/0.5 Sick LMS variant, a subrange over the scan area [0,10.5] would
   *          correspond to setting the index arguments as follows: sick_subrange_start = 1,
   *          sick_subrange_stop = 22
   */  
  void SickLMS2xx::GetSickMeanValuesSubrange( const uint8_t sick_sample_size,
					   const uint16_t sick_subrange_start_index,
					   const uint16_t sick_subrange_stop_index,
					   unsigned int * const measurement_values,
					   unsigned int & num_measurement_values,
					   unsigned int * const sick_telegram_index,
					   unsigned int * const sick_real_time_scan_index ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickMeanValuesSubrange: Sick LMS is not initialized!");
    }
    
    /* Declare message objects */
    SickLMS2xxMessage response;

    /* Declare some useful variables and a buffer */
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {
    
      /* Restore original operating mode */
      _setSickOpModeMonitorStreamMeanValuesSubrange(sick_sample_size,sick_subrange_start_index,sick_subrange_stop_index);
      
      /* Receive a data frame from the stream. */
      _recvMessage(response,DEFAULT_SICK_LMS_2XX_SICK_MEAN_VALUES_MESSAGE_TIMEOUT);

      /* Check that our payload has the proper command byte of 0xB0 */
      if(response.GetCommandCode() != 0xBF) {
	throw SickIOException("SickLMS2xx::GetSickMeanValuesSubrange: Unexpected message!");
      }

      /* Acquire the payload buffer and length*/
      response.GetPayload(payload_buffer);

      /* Define a local scan profile object */
      sick_lms_2xx_scan_profile_bf_t sick_scan_profile;

      /* Initialize the profile */
      memset(&sick_scan_profile,0,sizeof(sick_lms_2xx_scan_profile_bf_t));

      /* Parse the message payload */
      _parseSickScanProfileBF(&payload_buffer[1],sick_scan_profile);

      /* Return the request values! */
      num_measurement_values = sick_scan_profile.sick_num_measurements;

      for (unsigned int i = 0; i < num_measurement_values; i++) {

	/* Copy the measurement value */
	measurement_values[i] = sick_scan_profile.sick_measurements[i];

      }

      /* If requested, copy the real time scan index */
      if(sick_real_time_scan_index) {
	*sick_real_time_scan_index = sick_scan_profile.sick_real_time_scan_index;
      }
      
      /* If requested, copy the telegram index */
      if(sick_telegram_index) {
	*sick_telegram_index = sick_scan_profile.sick_telegram_index;
      }

    }

    /* Handle any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a timeout exception */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetMeanValuesSubrange: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Acquire the Sick LMS 2xx status
   * \return The status of the device
   *
   * NOTE: This method also updated the local view of all other information
   *       returned with a status request.
   */
  sick_lms_2xx_status_t SickLMS2xx::GetSickStatus( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::GetSickStatus: Sick LMS is not initialized!");
    }
    
    try {
      
      /* Refresh the status info! */
      _getSickStatus();

    }

    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
      
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::GetSickStatus: Unknown exception!" << std::endl;
      throw;
    }

    /* Return the latest Sick status */
    return (sick_lms_2xx_status_t)_sick_operating_status.sick_device_status;
  }

  /**
   * \brief Indicates whether the device is an LMS Fast
   */
  bool SickLMS2xx::IsSickLMS2xxFast() const throw(SickConfigException) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::IsSickLMS2xxFast: Sick LMS is not initialized!");
    }
    
    return (_sick_type == SICK_LMS_TYPE_211_S14 ||
	    _sick_type == SICK_LMS_TYPE_221_S14 ||
	    _sick_type == SICK_LMS_TYPE_291_S14);

  }
  
  /**
   * \brief Reset the Sick LMS 2xx active field values
   * NOTE: Considered successful if the LMS ready message is received.
   */
  void SickLMS2xx::ResetSick( ) throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    /* Ensure the device is initialized */
    if (!_sick_initialized) {
      throw SickConfigException("SickLMS2xx::ResetSick: Sick LMS is not initialized!");
    }
    
    SickLMS2xxMessage message,response;
    uint8_t payload[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Construct the reset command */
    payload[0] = 0x10; // Request field reset
    message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload,1);
    
    std::cout << "\tResetting the device..." << std::endl;
    std::cout << "\tWaiting for Power on message..." << std::endl;

    try {

      /* Send the reset command and wait for the reply */
      _sendMessageAndGetReply(message,response,0x91,(unsigned int)60e6,DEFAULT_SICK_LMS_2XX_NUM_TRIES);

      std::cout << "\t\tPower on message received!" << std::endl;
      std::cout << "\tWaiting for LMS Ready message..." << std::endl;

      /* Set terminal baud to the detected rate to get the LMS ready message */
      _setTerminalBaud(_baudToSickBaud(DEFAULT_SICK_LMS_2XX_SICK_BAUD));

      /* Receive the LMS ready message after power on */
      _recvMessage(response,(unsigned int)30e6);
      
      /* Verify the response */
      if(response.GetCommandCode() != 0x90) {
 	std::cerr << "SickLMS2xx::ResetSick: Unexpected reply! (assuming device has been reset!)"  << std::endl;
      } else {
 	std::cout << "\t\tLMS Ready message received!" << std::endl;
      }
      std::cout << std::endl;

      /* Reinitialize and sync the device */
      Initialize(_desired_session_baud);

    }
    
    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::ResetSick: Unknown exception!!!" << std::endl;
      throw;
    }
    
    std::cout << "\tRe-initialization sucessful. LMS is ready to go!" << std::endl;
    
  }

  /**
   * \brief Acquire the Sick LMS's status as a printable string
   * \return The Sick LMS status as a well-formatted string
   */
  std::string SickLMS2xx::GetSickStatusAsString( ) const {

    std::stringstream str_stream;

    str_stream << "\t=============== Sick LMS Status ===============" << std::endl;

    /* If Sick is initialized then print the status! */
    if (_sick_initialized) {

      str_stream << "\tVariant: " << _sickVariantToString(_sick_operating_status.sick_variant) << std::endl;
      str_stream << "\tSensor Status: " << SickStatusToString((sick_lms_2xx_status_t)_sick_operating_status.sick_device_status) << std::endl;
      str_stream << "\tScan Angle: " << GetSickScanAngle() << " (deg)" << std::endl;
      str_stream << "\tScan Resolution: " << GetSickScanResolution() << " (deg)" << std::endl;
      str_stream << "\tOperating Mode: " << SickOperatingModeToString(GetSickOperatingMode()) << std::endl;
      str_stream << "\tMeasuring Mode: " << SickMeasuringModeToString(GetSickMeasuringMode()) << std::endl;
      str_stream << "\tMeasuring Units: " << SickMeasuringUnitsToString(GetSickMeasuringUnits()) << std::endl;

    }
    else {
      
      str_stream << "\t Unknown (Device is not initialized)" << std::endl;

    }      

    str_stream << "\t===============================================" << std::endl;
    
    return str_stream.str();
  }

  /**
   * \brief Acquire the Sick LMS's operating params as a printable string
   * \return The Sick LMS operating params as a well-formatted string
   */
  std::string SickLMS2xx::GetSickSoftwareVersionAsString( ) const {

    std::stringstream str_stream;
    
    str_stream << "\t============== Sick LMS Software ==============" << std::endl;

    if (_sick_initialized) {
    
      str_stream << "\tSystem Software: " << std::string((char *)_sick_software_status.sick_system_software_version) << std::endl;
      str_stream << "\tSystem Boot PROM Software: " << std::string((char *)_sick_software_status.sick_prom_software_version) << std::endl;

    }
    else {
      
      str_stream << "\t Unknown (Device is not initialized)" << std::endl;
      
    }
    
    str_stream << "\t===============================================" << std::endl;

    return str_stream.str();
  }

  /**
   * \brief Acquire the Sick LMS's config as a printable string
   * \return The Sick LMS config as a well-formatted string
   */
  std::string SickLMS2xx::GetSickConfigAsString( ) const {

    std::stringstream str_stream;

    str_stream<< "\t=============== Sick LMS Config ===============" << std::endl;

    if (_sick_initialized) {
  
      str_stream << "\tBlanking Value: " << _sick_device_config.sick_blanking << std::endl;
      
      if(_isSickLMS211() || _isSickLMS221() || _isSickLMS291()) {
	str_stream << "\tSensitivity: " << SickSensitivityToString(GetSickSensitivity()) << std::endl;
      }
      else {
	str_stream << "\tPeak Thresh: " << SickPeakThresholdToString((sick_lms_2xx_peak_threshold_t)_sick_device_config.sick_peak_threshold) << std::endl;
	str_stream << "\tStop Thresh: " << (unsigned int)_sick_device_config.sick_stop_threshold << std::endl;
      }
    
      str_stream << "\tAvailability: " << _sickAvailabilityToString(_sick_device_config.sick_availability_level) << std::endl;
      str_stream << "\tMeasuring Mode: " << SickMeasuringModeToString((sick_lms_2xx_measuring_mode_t)_sick_device_config.sick_measuring_mode) << std::endl;
      str_stream << "\tMeasuring Units: " << SickMeasuringUnitsToString((sick_lms_2xx_measuring_units_t)_sick_device_config.sick_measuring_units) << std::endl;
      str_stream << "\tTemporary Field: " << _sickTemporaryFieldToString(_sick_device_config.sick_temporary_field) << std::endl;
      str_stream << "\tSubtractive Fields: " << _sickSubtractiveFieldsToString(_sick_device_config.sick_subtractive_fields) << std::endl;
      str_stream << "\tMultiple Evaluation: " << (unsigned int)_sick_device_config.sick_multiple_evaluation << std::endl;
      str_stream << "\tSuppressed Objects Multiple Evaluation: " << (unsigned int)_sick_device_config.sick_multiple_evaluation_suppressed_objects << std::endl;
      str_stream << "\tDazzling Multiple Evaluation: " << (unsigned int)_sick_device_config.sick_dazzling_multiple_evaluation << std::endl;
      str_stream << "\tRestart Mode: " << _sickRestartToString(_sick_device_config.sick_restart) << std::endl;
      str_stream << "\tRestart Time: " << (unsigned int)_sick_device_config.sick_restart_time << std::endl;
      str_stream << "\tFields B,C Restart Time: " << (unsigned int)_sick_device_config.sick_fields_b_c_restart_times << std::endl;
      str_stream << "\tContour Function A: " << _sickContourFunctionToString(_sick_device_config.sick_contour_a_reference) << std::endl;
      str_stream << "\tContour Function B: " << _sickContourFunctionToString(_sick_device_config.sick_contour_b_reference) << std::endl;
      str_stream << "\tContour Function C: " << _sickContourFunctionToString(_sick_device_config.sick_contour_c_reference) << std::endl;
      str_stream << "\tPixel Oriented Evaluation: " << (unsigned int)_sick_device_config.sick_pixel_oriented_evaluation << std::endl;
      str_stream << "\tSingle Measured Value Eval. Mode: " << (unsigned int)_sick_device_config.sick_single_measured_value_evaluation_mode << std::endl;
      
    }
    else {
      
      str_stream << "\t Unknown (Device is not initialized)" << std::endl;
      
    }

    str_stream << "\t===============================================" << std::endl;
    
    return str_stream.str();
  }
  
  /**
   * \brief Prints ths status of the Sick LMS 2xx unit
   */
  void SickLMS2xx::PrintSickStatus() const {
    std::cout << GetSickStatusAsString() << std::endl;
  }

  /**
   * \brief Prints out relevant software versioning information
   */
  void SickLMS2xx::PrintSickSoftwareVersion() const {
    std::cout << GetSickSoftwareVersionAsString() << std::endl;  
  }

  /**
   * \brief Prints out the Sick LMS configurations parameters
   */
  void SickLMS2xx::PrintSickConfig() const {
    std::cout << GetSickConfigAsString() << std::endl;
  }
  
  /**
   * \brief Converts the Sick LMS type to a corresponding string
   * \param sick_type The device type
   * \return Sick LMS type as a string
   */
  std::string SickLMS2xx::SickTypeToString( const sick_lms_2xx_type_t sick_type ) {

    switch(sick_type) {
    case SICK_LMS_TYPE_200_30106:
      return "Sick LMS 200-30106";
    case SICK_LMS_TYPE_211_30106:
      return "Sick LMS 211-30106";
    case SICK_LMS_TYPE_211_30206:
      return "Sick LMS 211-30206";
    case SICK_LMS_TYPE_211_S07:
      return "Sick LMS 211-S07";
    case SICK_LMS_TYPE_211_S14:
      return "Sick LMS 211-S14";
    case SICK_LMS_TYPE_211_S15:
      return "Sick LMS 211-S15";
    case SICK_LMS_TYPE_211_S19:
      return "Sick LMS 211-S19";
    case SICK_LMS_TYPE_211_S20:
      return "Sick LMS 211-S20";
    case SICK_LMS_TYPE_220_30106:
      return "Sick LMS 220-30106";
    case SICK_LMS_TYPE_221_30106:
      return "Sick LMS 221-30106";
    case SICK_LMS_TYPE_221_30206:
      return "Sick LMS 221-30206";
    case SICK_LMS_TYPE_221_S07:
      return "Sick LMS 221-S07";
    case SICK_LMS_TYPE_221_S14:
      return "Sick LMS 221-S14";
    case SICK_LMS_TYPE_221_S15:
      return "Sick LMS 221-S15";
    case SICK_LMS_TYPE_221_S16:
      return "Sick LMS 221-S16";
    case SICK_LMS_TYPE_221_S19:
      return "Sick LMS 221-S19";
    case SICK_LMS_TYPE_221_S20:
      return "Sick LMS 221-S20";
    case SICK_LMS_TYPE_291_S05:
      return "Sick LMS 291-S05";
    case SICK_LMS_TYPE_291_S14:
      return "Sick LMS 291-S14";
    case SICK_LMS_TYPE_291_S15:
      return "Sick LMS 291-S15";
    default:
      return "Unknown!";
    }
    
  }

  /**
   * \brief Converts integer to corresponding Sick LMS scan angle
   * \param scan_angle_int Scan angle (FOV) as an integer (e.g. 90,100,180)
   */
  sick_lms_2xx_scan_angle_t SickLMS2xx::IntToSickScanAngle( const int scan_angle_int ) {

    switch(scan_angle_int) {
    case 90:
      return SICK_SCAN_ANGLE_90;
    case 100:
      return SICK_SCAN_ANGLE_100;
    case 180:
      return SICK_SCAN_ANGLE_180;
    default:
      return SICK_SCAN_ANGLE_UNKNOWN;      
    }

  }
  
  /**
   * \brief Converts integer to corresponding Sick LMS scan resolution
   * \param scan_resolution_int Scan resolution as an integer (e.g. 25,50,100)
   */
  sick_lms_2xx_scan_resolution_t SickLMS2xx::IntToSickScanResolution( const int scan_resolution_int ) {

    switch(scan_resolution_int) {
    case 25:
      return SICK_SCAN_RESOLUTION_25;
    case 50:
      return SICK_SCAN_RESOLUTION_50;
    case 100:
      return SICK_SCAN_RESOLUTION_100;
    default:
      return SICK_SCAN_RESOLUTION_UNKNOWN;      
    }

  }
  
  /**
   * \brief Converts double to corresponding Sick LMS scan resolution
   * \param scan_resolution_double Scan resolution as a double (e.g. 0.25,0.5,1.0)
   */
  sick_lms_2xx_scan_resolution_t SickLMS2xx::DoubleToSickScanResolution( const double scan_resolution_double ) {
    return IntToSickScanResolution((const int)(scan_resolution_double*100));
  }
  
  /**
   * \brief Converts Sick LMS baud to a corresponding string
   * \param baud_rate The baud rate to be represented as a string
   * \return The string representation of the baud rate
   */
  std::string SickLMS2xx::SickBaudToString( const sick_lms_2xx_baud_t baud_rate ) {

    switch(baud_rate) {
    case SICK_BAUD_9600:
      return "9600bps";
    case SICK_BAUD_19200:
      return "19200bps";
    case SICK_BAUD_38400:
      return "38400bps";
    case SICK_BAUD_500K:
      return "500Kbps";
    default:
      return "Unknown!";
    }
    
  }

  /**
   * \brief Converts integer to corresponding Sick LMS baud
   * \param baud_str Baud rate as integer (e.g. 9600,19200,38400,500000)
   */
  sick_lms_2xx_baud_t SickLMS2xx::IntToSickBaud( const int baud_int ) {

    switch(baud_int) {
    case 9600:
      return SICK_BAUD_9600;
    case 19200:
      return SICK_BAUD_19200;
    case 38400:
      return SICK_BAUD_38400;
    case 500000:
      return SICK_BAUD_500K;
    default:
      return SICK_BAUD_UNKNOWN;      
    }

  }
  
  /**
   * \brief Converts string to corresponding Sick LMS baud
   * \param baud_str Baud rate as string (e.g. "9600","19200","38400","500000")
   */
  sick_lms_2xx_baud_t SickLMS2xx::StringToSickBaud( const std::string baud_str ) {

    int baud_int;
    std::istringstream input_stream(baud_str);
    input_stream >> baud_int;
    
    return IntToSickBaud(baud_int);

  }
  
  /**
   * \brief Converts the Sick LMS 2xx status code to a string
   * \param sick_status The device status
   * \return A string corresponding to the given status code
   */
  std::string SickLMS2xx::SickStatusToString( const sick_lms_2xx_status_t sick_status ) {

    /* Return a string */
    if(sick_status != SICK_STATUS_OK) {
      return "Error (possibly fatal)";
    }  
    return "OK!";
    
  }

  /**
   * \brief Converts the Sick measuring mode to a corresponding string
   * \param sick_measuring_mode The Sick measuring mode
   * \return The corresponding string
   */
  std::string SickLMS2xx::SickMeasuringModeToString( const sick_lms_2xx_measuring_mode_t sick_measuring_mode ) {

    switch(sick_measuring_mode) {
    case SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE:
      return "8m/80m; fields A,B,Dazzle";
    case SICK_MS_MODE_8_OR_80_REFLECTOR:
      return "8m/80m; 3 reflector bits";
    case SICK_MS_MODE_8_OR_80_FA_FB_FC:
      return "8m/80m; fields A,B,C";
    case SICK_MS_MODE_16_REFLECTOR:
      return "16m; 4 reflector bits";
    case SICK_MS_MODE_16_FA_FB:
      return "16m; fields A & B";
    case SICK_MS_MODE_32_REFLECTOR:
      return "32m; 2 reflector bits";
    case SICK_MS_MODE_32_FA:
      return "32m; field A";
    case SICK_MS_MODE_32_IMMEDIATE:
      return "32m; immediate";
    case SICK_MS_MODE_REFLECTIVITY:
      return "Reflectivity";
    default:
      return "Unknown";
    }  
  }

  /**
   * \brief Converts the Sick operating mode to a corresponding string
   * \param sick_operating_mode The Sick operating mode
   * \return The corresponding string
   */
  std::string SickLMS2xx::SickOperatingModeToString( const sick_lms_2xx_operating_mode_t sick_operating_mode ) {

    switch(sick_operating_mode) {
    case SICK_OP_MODE_INSTALLATION:
      return "Installation Mode";
    case SICK_OP_MODE_DIAGNOSTIC:
      return "Diagnostic Mode";
    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT:
      return "Stream mim measured values for each segment";
    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT:
      return "Min measured value for each segment when object detected";
    case SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT:
      return "Min vertical distance";
    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT:
      return "Min vertical distance when object detected";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES:
      return "Stream all measured values";
    case SICK_OP_MODE_MONITOR_REQUEST_VALUES:
      return "Request measured values";
    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES:
      return "Stream mean measured values";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE:
      return "Stream measured value subrange";
    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE:
      return "Stream mean measured value subrange";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS:
      return "Stream measured and field values";
    case SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN:
      return "Stream measured values from partial scan";
    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN:
      return "Stream range w/ reflectivity from partial scan";
    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE:
      return "Stream min measured values for each segment over a subrange";
    case SICK_OP_MODE_MONITOR_NAVIGATION:
      return "Output navigation data records";
    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT:
      return "Stream range w/ reflectivity values";
    default:
      return "Unknown!";
    };
    
  }
  
  /**
   * \brief Converts Sick LMS 2xx sensitivity level to a corresponding string
   * \param sick_sensitivity Sick sensitivity level
   * \return The corresponding string
   */
  std::string SickLMS2xx::SickSensitivityToString( const sick_lms_2xx_sensitivity_t sick_sensitivity ) {

    switch(sick_sensitivity) {    
    case SICK_SENSITIVITY_STANDARD:
      return "Standard (~30m @ 10% reflectivity)";
    case SICK_SENSITIVITY_MEDIUM:
      return "Medium (~25m @ 10% reflectivity)";
    case SICK_SENSITIVITY_LOW:
      return "Low (~20m @ 10% relfectivity)";
    case SICK_SENSITIVITY_HIGH:
      return "High (~42m @ 10% reflectivity)";
    default:
      return "Unknown!";
    }

  }

  /**
   * \brief Converts Sick LMS 2xx peak threshold to a corresponding string
   * \param sick_peak_threshold Sick sensitivity level
   * \return The corresponding string
   */
  std::string SickLMS2xx::SickPeakThresholdToString( const sick_lms_2xx_peak_threshold_t sick_peak_threshold ) {

    switch(sick_peak_threshold) {    
    case SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION:
      return "Peak detection, no black extension";
    case SICK_PEAK_THRESHOLD_DETECTION_WITH_BLACK_EXTENSION:
      return "Peak detection w/ black extension";
    case SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_NO_BLACK_EXTENSION:
      return "No peak detection, no black extension";
    case SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_BLACK_EXTENSION:
      return "No peak detection w/ black extension";
    default:
      return "Unknown!";
    }

  }
  
  /**
   * \brief Converts the Sick LMS measurement units to a corresponding string
   * \param sick_units The measuring units
   * \return The corresponding string
   */
  std::string SickLMS2xx::SickMeasuringUnitsToString( const sick_lms_2xx_measuring_units_t sick_units ) {

    /* Return the proper string */
    switch(sick_units) {
    case SICK_MEASURING_UNITS_CM:
      return "Centimeters (cm)";
    case SICK_MEASURING_UNITS_MM:
      return "Millimeters (mm)";
    default:
      return "Unknown!";
    }
    
  }

  /**
   * \brief Attempts to open a I/O stream using the device path given at object instantiation.
   */
   void SickLMS2xx::_setupConnection() throw ( SickIOException, SickThreadException ) {
     SickLMS2xx::_setupConnection(0);
   }
  
  /**
   * \brief Attempts to open a I/O stream using the device path given at object instantiation
   * \param delay Delay to wait for SICK to power on. (In seconds)
   */
  void SickLMS2xx::_setupConnection( const uint32_t delay ) throw ( SickIOException, SickThreadException ) {

    try {
    
      /* Open the device */
      if((_sick_fd = open(_sick_device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
	throw SickIOException("SickLMS2xx::_setupConnection: - Unable to open serial port");
      }

      // Sleep to allow the SICK to power on for some applications
      sleep(delay);
      
      /* Backup the original term settings */
      if(tcgetattr(_sick_fd,&_old_term) < 0) {
	throw SickIOException("SickLMS2xx::_setupConnection: tcgetattr() failed!");
      }

      /* Set the host terminal baud rate to the new speed */
      _setTerminalBaud(_baudToSickBaud(DEFAULT_SICK_LMS_2XX_SICK_BAUD));
      
    }

    /* Handle any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Handle unknown exceptions */
    catch(...) {
      std::cerr << "SickLMS2xx::_setupConnection: Unknown exception!" << std::endl;
      throw;
    }
    
  }

  /**
   * \brief Closes the data connection associated with the device
   */
  void SickLMS2xx::_teardownConnection( ) throw( SickIOException ) {

    /* Check whether device was initialized */
    if(!_sick_initialized) {
      return;
    }
    
    /* Restore old terminal settings */
    if (tcsetattr(_sick_fd,TCSANOW,&_old_term) < 0) {
      throw SickIOException("SickLMS2xx::_teardownConnection: tcsetattr() failed!");
    }

    /* Actually close the device */
    if(close(_sick_fd) != 0) {
      throw SickIOException("SickLMS2xx::_teardownConnection: close() failed!");
    }

  }

  /**
   * \brief Flushes terminal I/O buffers
   */
  void SickLMS2xx::_flushTerminalBuffer( ) throw ( SickThreadException ) {

    try {
    
      /* Acquire access to the data stream */    
      _sick_buffer_monitor->AcquireDataStream();

      /* Nobody is reading a message, so safely flush! */
      if (tcflush(_sick_fd,TCIOFLUSH) != 0) {
      	throw SickThreadException("SickLMS2xx::_flushTerminalBuffer: tcflush() failed!");
      }
      
      /* Attempt to release the data stream */
      _sick_buffer_monitor->ReleaseDataStream();

    }

    /* Handle thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* A sanity check */
    catch(...) {
      std::cerr << "SickLMS2xx::_flushTerminalBuffer: Unknown exception!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Sends a message and searches for the corresponding reply
   * \param &send_message The message to be sent to the Sick LMS 2xx unit
   * \param &recv_message The expected message reply from the Sick LMS
   * \param timeout_value The epoch to wait before considering a sent frame lost (in usecs)
   * \param num_tries The number of times to try and transmit the message
   *                  before quitting
   *
   * NOTE: Uses the 0x80 response code rule for looking for the response message
   */
  void SickLMS2xx::_sendMessageAndGetReply( const SickLMS2xxMessage &send_message,
					    SickLMS2xxMessage &recv_message,
					    const unsigned int timeout_value,
					    const unsigned int num_tries ) throw( SickIOException, SickThreadException, SickTimeoutException ) {

    uint8_t sick_reply_code = send_message.GetCommandCode() + 0x80;
    
    try {

      /* Send a message and get reply using a reply code */
      _sendMessageAndGetReply(send_message,recv_message,sick_reply_code,timeout_value,num_tries);

    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout) {
      /* For now just rethrow it */
      throw;
    }
    
    /* Handle a thread exception */
    catch (SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_error) {
      std::cerr << sick_io_error.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS2xx::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Sends a message and searches for the reply with given reply code
   * \param &send_message The message to be sent to the Sick LMS 2xx unit
   * \param &recv_message The expected message reply from the Sick LMS
   * \param reply_code The reply code associated with the expected messgage
   * \param timeout_value The epoch to wait before considering a sent frame lost (in usecs)
   * \param num_tries The number of times to send the message in the event the LMS fails to reply
   */
  void SickLMS2xx::_sendMessageAndGetReply( const SickLMS2xxMessage &send_message,
					    SickLMS2xxMessage &recv_message,
					    const uint8_t reply_code,
					    const unsigned int timeout_value,
					    const unsigned int num_tries ) throw( SickIOException, SickThreadException, SickTimeoutException ) {

    try {

      /* Attempt to flush the terminal buffer */
      _flushTerminalBuffer();
      
      /* Send a message and get reply using parent's method */
      SickLIDAR< SickLMS2xxBufferMonitor, SickLMS2xxMessage >::_sendMessageAndGetReply(send_message,recv_message,&reply_code,1,DEFAULT_SICK_LMS_2XX_BYTE_INTERVAL,timeout_value,num_tries);

    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout) {
      throw;
    }
    
    /* Handle a thread exception */
    catch (SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_error) {
      std::cerr << sick_io_error.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS2xx::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Sets the baud rate for the current communication session
   * \param baud_rate The desired baud rate
   */
  void SickLMS2xx::_setSessionBaud(const sick_lms_2xx_baud_t baud_rate) throw ( SickIOException, SickThreadException, SickTimeoutException ){
    
    SickLMS2xxMessage message, response;
    
    uint8_t payload[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    /* Another sanity check */
    if(baud_rate == SICK_BAUD_UNKNOWN) {
      throw SickIOException("SickLMS2xx::_setSessionBaud: Undefined baud rate!");
    }    
    
    /* Construct the command telegram */
    payload[0] = 0x20;
    payload[1] = baud_rate;
    
    message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload,2);
    
    try {

      /* Send the status request and get a reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);

      /* Set the host terminal baud rate to the new speed */
      _setTerminalBaud(baud_rate);

      /* Sick likes a sleep here */
      usleep(250000);
      
    }
    
    /* Catch a timeout */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::_getSickErrors: Unknown exception!!!" << std::endl;
      throw;
    }

  }
  
  /**
   * \brief Attempts to detect whether the LMS is operating at the given baud rate
   * \param baud_rate The baud rate to use when "pinging" the Sick LMS 2xx
   */
  bool SickLMS2xx::_testSickBaud(const sick_lms_2xx_baud_t baud_rate) throw( SickIOException, SickThreadException ) {

    try {
    
      /* Another sanity check */
      if(baud_rate == SICK_BAUD_UNKNOWN) {
	throw SickIOException("SickLMS2xx::_testBaudRate: Undefined baud rate!");
      }
      
      /* Attempt to get status information at the current baud */
      std::cout << "\t\tChecking " << SickBaudToString(baud_rate) << "..." << std::endl;
      
      /* Set the host terminal baud rate to the test speed */
      _setTerminalBaud(baud_rate);
      
      try {

	/* Check to see if the Sick replies! */
	_getSickErrors();

      }

      /* Catch a timeout exception */
      catch(SickTimeoutException &sick_timeout_exception) {
	/* This means that the current baud rate timed out! */
	return false;
      }

      /* Catch anything else and throw it away */
      catch(...) {
	std::cerr << "SickLMS2xx::_testBaudRate: Unknown exception!" << std::endl;
	throw;
      }
      
    }

    /* Handle any IO exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Handle thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw; 
    }

    /* A safety net */
    catch(...) {
      std::cerr << "SickLMS2xx::_testBaudRate: Unknown exception!!!" << std::endl;
      throw; 
    }

    /* Success! */
    return true;

  }

  /**
   * \brief Sets the local terminal baud rate
   * \param baud_rate The desired terminal baud rate
   */
  void SickLMS2xx::_setTerminalBaud( const sick_lms_2xx_baud_t baud_rate ) throw( SickIOException, SickThreadException ) {

    struct termios term;

#ifdef HAVE_LINUX_SERIAL_H
    struct serial_struct serial;
#endif
    
    try {
    
      /* If seeting baud to 500k */
      if (baud_rate == SICK_BAUD_500K) {

#ifdef HAVE_LINUX_SERIAL_H
	
	/* Get serial attributes */
	if(ioctl(_sick_fd,TIOCGSERIAL,&serial) < 0) {
	  throw SickIOException("SickLMS2xx::_setTerminalBaud: ioctl() failed!");
	}
	
	/* Set the custom devisor */
	serial.flags |= ASYNC_SPD_CUST;
	serial.custom_divisor = 48; // for FTDI USB/serial converter divisor is 240/5
	
	/* Set the new attibute values */
	if(ioctl(_sick_fd,TIOCSSERIAL,&serial) < 0) {
	  throw SickIOException("SickLMS2xx::_setTerminalBaud: ioctl() failed!");
	}

#else
	throw SickIOException("SickLMS2xx::_setTerminalBaud - 500K baud is only supported under Linux!");
#endif
	
      }

#ifdef HAVE_LINUX_SERIAL_H
      
      else { /* Using a standard baud rate */

	/* We let the next few errors slide in case USB adapter is being used */
	if(ioctl(_sick_fd,TIOCGSERIAL,&serial) < 0) {
	  std::cerr << "SickLMS2xx::_setTermSpeed: ioctl() failed while trying to get serial port info!" << std::endl;
	  std::cerr << "\tNOTE: This is normal when connected via USB!" <<std::endl;
	}
	
	serial.custom_divisor = 0;
        serial.flags &= ~ASYNC_SPD_CUST;
	
	if(ioctl(_sick_fd,TIOCSSERIAL,&serial) < 0) {
	  std::cerr << "SickLMS2xx::_setTerminalBaud: ioctl() failed while trying to set serial port info!" << std::endl;
	  std::cerr << "\tNOTE: This is normal when connected via USB!" <<std::endl;
	}
	
      }
      
#endif
      
      /* Attempt to acquire device attributes */
      if(tcgetattr(_sick_fd,&term) < 0) {
	throw SickIOException("SickLMS2xx::_setTerminalBaud: Unable to get device attributes!");
      }
      
      /* Switch on the baud rate */
      switch(baud_rate) {      
      case SICK_BAUD_9600: {
	cfmakeraw(&term);
	cfsetispeed(&term,B9600);
	cfsetospeed(&term,B9600);
	break;
      }
      case SICK_BAUD_19200: {
	cfmakeraw(&term);
	cfsetispeed(&term,B19200);
	cfsetospeed(&term,B19200);
	break;
      }
      case SICK_BAUD_38400: {
	cfmakeraw(&term);
	cfsetispeed(&term,B38400);
	cfsetospeed(&term,B38400);            
	break;
      }
      case SICK_BAUD_500K: {      
	cfmakeraw(&term);
	cfsetispeed(&term,B38400);
	cfsetospeed(&term,B38400);
	break;
      }
      default:
	throw SickIOException("SickLMS2xx::_setTerminalBaud: Unknown baud rate!");
      }
      
      /* Attempt to set the device attributes */
      if(tcsetattr(_sick_fd,TCSAFLUSH,&term) < 0 ) {
	throw SickIOException("SickLMS2xx::_setTerminalBaud: Unable to set device attributes!");
      }
      
      /* Buffer the rate locally */
      _curr_session_baud = baud_rate;
      
      /* Attempt to flush the I/O buffers */
      _flushTerminalBuffer();
      
    } // try

    /* Catch an IO exception */
    catch(SickIOException sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch an IO exception */
    catch(SickThreadException sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* A sanity check */
    catch(...) {
      std::cerr << "SickLMS2xx::_setTerminalBaud: Unknown exception!!!" << std::endl;
      throw;
    }

  }

  /**
   * \brief Acquires the sick device type (as a string) from the unit
   */
  void SickLMS2xx::_getSickType( ) throw( SickTimeoutException, SickIOException, SickThreadException ) {
    
    SickLMS2xxMessage message,response;
    
    int payload_length;
    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    /* Get the LMS type */
    payload_buffer[0] = 0x3A; //Command to request LMS type
    
    /* Build the message */
    message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,1);

    try {
       
      /* Send the status request and get a reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);
      
    }
    
    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::_getSickType: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer */
    memset(payload_buffer,0,1);
  
    /* Get the payload */
    response.GetPayload(payload_buffer);
    
    /* Acquire the payload length */
    payload_length = response.GetPayloadLength();
    
    /* Dynamically allocate the string length */
    char * string_buffer = new char[payload_length-1];

    /* Initialize the buffer */
    memset(string_buffer,0,payload_length-1);
    memcpy(string_buffer,&payload_buffer[1],payload_length-2);

    /* Convert to a standard string */
    std::string type_string = string_buffer;

    /* Set the Sick LMS type in the driver */
    if(type_string.find("LMS200;30106") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_200_30106;
    } else if(type_string.find("LMS211;30106") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_30106;
    } else if(type_string.find("LMS211;30206") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_30206;
    } else if(type_string.find("LMS211;S07") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_S07;
    } else if(type_string.find("LMS211;S14") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_S14;
    } else if(type_string.find("LMS211;S15") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_S15;
    } else if(type_string.find("LMS211;S19") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_S19;
    } else if(type_string.find("LMS211;S20") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_211_S20;
    } else if(type_string.find("LMS220;30106") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_220_30106;
    } else if(type_string.find("LMS221;30106") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_30106;
    } else if(type_string.find("LMS221;30206") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_30206;
    } else if(type_string.find("LMS221;S07") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_S07;
    } else if(type_string.find("LMS221;S14") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_S14;
    } else if(type_string.find("LMS221;S15") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_S15;
    } else if(type_string.find("LMS221;S16") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_S16;
    } else if(type_string.find("LMS221;S19") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_S19;
    } else if(type_string.find("LMS221;S20") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_221_S20;
    } else if(type_string.find("LMS291;S05") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_291_S05;
    } else if(type_string.find("LMS291;S14") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_291_S14;
    } else if(type_string.find("LMS291;S15") != std::string::npos) {
      _sick_type = SICK_LMS_TYPE_291_S15;
    } else {
      _sick_type = SICK_LMS_TYPE_UNKNOWN;
    }

    /* Reclaim the allocated string buffer */
    if (string_buffer) {
      delete [] string_buffer;
    }
    
  }

  /**
   * \brief Acquires (and buffers) the current Sick LMS configuration from the device
   */
  void SickLMS2xx::_getSickConfig( ) throw( SickTimeoutException, SickIOException, SickThreadException ) {

     SickLMS2xxMessage message, response;

     uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};    

     /* Set the command code */
     payload_buffer[0] = 0x74;

     /* Build the request message */
     message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,1);

     try {
       
       /* Send the status request and get a reply */
       _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);
       
     }

     /* Catch any timeout exceptions */
     catch(SickTimeoutException &sick_timeout_exception) {
       std::cerr << sick_timeout_exception.what() << std::endl;
       throw;
     }
     
     /* Catch any I/O exceptions */
     catch(SickIOException &sick_io_exception) {
       std::cerr << sick_io_exception.what() << std::endl;
       throw;
     }
     
     /* Catch any thread exceptions */
     catch(SickThreadException &sick_thread_exception) {
       std::cerr << sick_thread_exception.what() << std::endl;
       throw;
     }
     
     /* Catch anything else */
     catch(...) {
       std::cerr << "SickLMS2xx::_getSickConfig: Unknown exception!!!" << std::endl;
       throw;
     }

     /* Reset the payload buffer */
     payload_buffer[0] = 0;

     /* Extract the payload */
     response.GetPayload(payload_buffer);

     /* Obtain the configuration results */
     _parseSickConfigProfile(&payload_buffer[1],_sick_device_config);
     
  }

  /**
   * \brief Sets the current configuration in flash
   * \param &sick_device_config The desired Sick LMS configuration
   */
  void SickLMS2xx::_setSickConfig( const sick_lms_2xx_device_config_t &sick_device_config )
    throw( SickConfigException, SickTimeoutException, SickIOException, SickThreadException ) {

    try {
      
      std::cout << "\tAttempting to configure the device (this can take a few seconds)..." << std::endl;
      
      /* Attempt to set the Sick into installation mode */
      _setSickOpModeInstallation();
      
      /* Define our message objects */
      SickLMS2xxMessage message, response;    
      uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};    
      
      /* Set the command code */
      payload_buffer[0] = 0x77; // Command to configure device
      
      /*
       * Build the corresponding config message
       */
      
      /* Set Block A */
      uint16_t temp_buffer = host_to_sick_lms_2xx_byte_order(sick_device_config.sick_blanking);
      memcpy(&payload_buffer[1],&temp_buffer,2);
      
      /* Set Block B */
      payload_buffer[3] = sick_device_config.sick_stop_threshold; // NOTE: This value will be 0 for LMS 211/221/291
      payload_buffer[4] = sick_device_config.sick_peak_threshold; // NOTE: This value is equivalent to sensitivity field on 211/221/291
  
      /* Set Block C */
      payload_buffer[5] = sick_device_config.sick_availability_level;
      
      /* Set Block D */
      payload_buffer[6] = sick_device_config.sick_measuring_mode;
      
      /* Set Block E */
      payload_buffer[7] = sick_device_config.sick_measuring_units;
      
      /* Set Block F */
      payload_buffer[8] = sick_device_config.sick_temporary_field;
      
      /* Set Block G */
      payload_buffer[9] = sick_device_config.sick_subtractive_fields;
      
      /* Set Block H */
      payload_buffer[10] = sick_device_config.sick_multiple_evaluation;
      
      /* Set Block I */
      payload_buffer[11] = sick_device_config.sick_restart;
      
      /* Set Block J */
      payload_buffer[12] = sick_device_config.sick_restart_time;
      
      /* Set Block K */
      payload_buffer[13] = sick_device_config.sick_multiple_evaluation_suppressed_objects;
      
      /* Set Block L */
      payload_buffer[14] = sick_device_config.sick_contour_a_reference;
    
      /* Set Block M */
      payload_buffer[15] = sick_device_config.sick_contour_a_positive_tolerance_band;
      
      /* Set Block N */
      payload_buffer[16] = sick_device_config.sick_contour_a_negative_tolerance_band;
      
      /* Set Block O */
      payload_buffer[17] = sick_device_config.sick_contour_a_start_angle;
      
      /* Set Block P */
      payload_buffer[18] = sick_device_config.sick_contour_a_stop_angle;
      
      /* Set Block Q */
      payload_buffer[19] = sick_device_config.sick_contour_b_reference;
      
      /* Set Block R */
      payload_buffer[20] = sick_device_config.sick_contour_b_positive_tolerance_band;
      
      /* Set Block S */
      payload_buffer[21] = sick_device_config.sick_contour_b_negative_tolerance_band;
      
      /* Set Block T */
      payload_buffer[22] = sick_device_config.sick_contour_b_start_angle;
    
      /* Set Block U */
      payload_buffer[23] = sick_device_config.sick_contour_b_stop_angle;
      
      /* Set Block V */
      payload_buffer[24] = sick_device_config.sick_contour_c_reference;
      
      /* Set Block W */
      payload_buffer[25] = sick_device_config.sick_contour_c_positive_tolerance_band;
      
      /* Set Block X */
      payload_buffer[26] = sick_device_config.sick_contour_c_negative_tolerance_band;
      
      /* Set Block Y */
      payload_buffer[27] = sick_device_config.sick_contour_c_start_angle;
      
      /* Set Block Z */
      payload_buffer[28] = sick_device_config.sick_contour_c_stop_angle;
      
      /* Set Block A1 */
      payload_buffer[29] = sick_device_config.sick_pixel_oriented_evaluation;
      
      /* Set Block A2 */
      payload_buffer[30] = sick_device_config.sick_single_measured_value_evaluation_mode;
      
      /* Set Block A3 */
      temp_buffer = host_to_sick_lms_2xx_byte_order(sick_device_config.sick_fields_b_c_restart_times);
      memcpy(&payload_buffer[31],&temp_buffer,2);
      
      /* Set Block A4 */
      temp_buffer = host_to_sick_lms_2xx_byte_order(sick_device_config.sick_dazzling_multiple_evaluation);
      memcpy(&payload_buffer[33],&temp_buffer,2);
      
      /* Populate the message container */
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,35);
      
      /* Send the status request and get a reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_CONFIG_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);

      /* Reset the payload buffer */
      memset(payload_buffer,0,35);

      /* Extract the payload contents */
      response.GetPayload(payload_buffer);

      /* Check whether the configuration was successful */
      if (payload_buffer[1] != 0x01) {
	throw SickConfigException("SickLMS2xx::_setSickConfig: Configuration failed!");
      }

      /* Success */
      std::cout << "\t\tConfiguration successful! :o)" << std::endl;

      /* Update the local configuration data */
      _parseSickConfigProfile(&payload_buffer[2],_sick_device_config);    
      
      /* Set the device back to request range mode */
      _setSickOpModeMonitorRequestValues();

      /* Refresh the status info! */
      _getSickStatus();
      
    }
    
    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    /* Catch any config exceptions */
    catch(SickConfigException &sick_config_exception) {
      std::cerr << sick_config_exception.what() << std::endl;
      throw;
    }
      
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::_setSickConfig: Unknown exception!" << std::endl;
      throw;
    }

  }
  
  /**
   * \brief Obtains any error codes from the Sick LMS
   */
  void SickLMS2xx::_getSickErrors( unsigned int * const num_sick_errors, uint8_t * const error_type_buffer,
				uint8_t * const error_num_buffer ) throw( SickTimeoutException, SickIOException, SickThreadException ) {

     SickLMS2xxMessage message, response;

     int payload_length;
     uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
  
     /* The command to request LMS status */
     payload_buffer[0] = 0x32;
     
     /* Build the request message */
     message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,1);
     
     try {
       
       /* Send the status request and get a reply */
       _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);
       
     }

     /* Catch any timeout exceptions */
     catch(SickTimeoutException &sick_timeout_exception) {
       std::cerr << sick_timeout_exception.what() << std::endl;
       throw;
     }
     
     /* Catch any I/O exceptions */
     catch(SickIOException &sick_io_exception) {
       std::cerr << sick_io_exception.what() << std::endl;
       throw;
     }
     
     /* Catch any thread exceptions */
     catch(SickThreadException &sick_thread_exception) {
       std::cerr << sick_thread_exception.what() << std::endl;
       throw;
     }
     
     /* Catch anything else */
     catch(...) {
       std::cerr << "SickLMS2xx::_getSickErrors: Unknown exception!!!" << std::endl;
       throw;
     }
     
     /* Extract the payload_length */
     payload_length = response.GetPayloadLength();
     
     /* Compute the number of errors */
     double num_errors = ((payload_length-2)/((double)2));
     
     /* Assign the number of errors if necessary */
     if (num_sick_errors) {
       *num_sick_errors = (unsigned int)num_errors;
     }
     
     /* Populate the return buffers with the error data */
     for (unsigned int i = 0, k = 1; i < (unsigned int)num_errors && (error_type_buffer || error_num_buffer); i++) {
       
       /* Check if the error type has been requested */
       if (error_type_buffer) {
	 error_type_buffer[i] = payload_buffer[k];
       }
       k++;
       
       /* Check if the error number has been requested */
       if (error_num_buffer) {
	 error_num_buffer[i] = payload_buffer[k];
       }
       k++;
       
     }

  }
  
  /**
   * \brief Acquires (and buffers) the status of the Sick LMS 2xx
   */
  void SickLMS2xx::_getSickStatus( ) throw( SickTimeoutException, SickIOException, SickThreadException ) {

    SickLMS2xxMessage message,response;

    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* The command to request LMS status */
    payload_buffer[0] = 0x31;

    /* Build the request message */
    message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,1);

    try {
    
      /* Send the status request and get a reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_MESSAGE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);

    }
    
    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::_getSickStatus: Unknown exception!" << std::endl;
      throw;
    }

    /* Reset the payload buffer */
    payload_buffer[0] = 0;

    /* Extract the payload contents */
    response.GetPayload(payload_buffer);
    
    /*
     * Extract the current Sick LMS operating config
     */

    /* Buffer the Sick LMS operating mode */
    _sick_operating_status.sick_operating_mode = payload_buffer[8];
    
    /* Buffer the status code */
    _sick_operating_status.sick_device_status = (payload_buffer[9]) ? SICK_STATUS_ERROR : SICK_STATUS_OK;
    
    /* Buffer the number of motor revolutions */
    memcpy(&_sick_operating_status.sick_num_motor_revs,&payload_buffer[67],2);
    _sick_operating_status.sick_num_motor_revs = sick_lms_2xx_to_host_byte_order(_sick_operating_status.sick_num_motor_revs);
    
    /* Buffer the measuring mode of the device */
    _sick_operating_status.sick_measuring_mode = payload_buffer[102];
    
    /* Buffer the scan angle of the device */
    memcpy(&_sick_operating_status.sick_scan_angle,&payload_buffer[107],2);
    _sick_operating_status.sick_scan_angle =
      sick_lms_2xx_to_host_byte_order(_sick_operating_status.sick_scan_angle);
    
    /* Buffer the angular resolution of the device */
    memcpy(&_sick_operating_status.sick_scan_resolution,&payload_buffer[109],2);
    _sick_operating_status.sick_scan_resolution =
      sick_lms_2xx_to_host_byte_order(_sick_operating_status.sick_scan_resolution);

    /* Buffer the variant type */
    _sick_operating_status.sick_variant = payload_buffer[18];
    
    /* Buffer the Sick LMS address */
    _sick_operating_status.sick_address = payload_buffer[120];
    
    /* Buffer the current measured value unit */
    _sick_operating_status.sick_measuring_units = payload_buffer[122];
    
    /* Buffer the laser switch flag */
    _sick_operating_status.sick_laser_mode = payload_buffer[123];

    
    /*
     * Extract the current Sick LMS software config
     */
    
    /* Buffer the software version string */
    memcpy(_sick_software_status.sick_system_software_version,&payload_buffer[1],7);

    /* Buffer the boot prom software version */
    memcpy(_sick_software_status.sick_prom_software_version,&payload_buffer[124],7);

    /*
     * Extract the Sick LMS restart config
     */

    /* Buffer the restart mode of the device */
    _sick_restart_status.sick_restart_mode = payload_buffer[111];
    
    /* Buffer the restart time of the device */
    memcpy(&_sick_restart_status.sick_restart_time,&payload_buffer[112],2);
    _sick_restart_status.sick_restart_time =
      sick_lms_2xx_to_host_byte_order(_sick_restart_status.sick_restart_time);
    
    /*
     * Extract the Sick LMS pollution status
     */

    /* Buffer the pollution values */
    for (unsigned int i = 0, k = 19; i < 8; i++, k+=2) {
      memcpy(&_sick_pollution_status.sick_pollution_vals[i],&payload_buffer[k],2);
      _sick_pollution_status.sick_pollution_vals[i] =
	sick_lms_2xx_to_host_byte_order(_sick_pollution_status.sick_pollution_vals[i]);
    }

    /* Buffer the reference pollution values */
    for (unsigned int i = 0, k = 35; i < 4; i++, k+=2) {
      memcpy(&_sick_pollution_status.sick_reference_pollution_vals[i],&payload_buffer[k],2);
      _sick_pollution_status.sick_reference_pollution_vals[i] =
	sick_lms_2xx_to_host_byte_order(_sick_pollution_status.sick_reference_pollution_vals[i]);
    }
    
    /* Buffer the calibrating pollution values */
    for (unsigned int i = 0, k = 43; i < 8; i++, k+=2) {
      memcpy(&_sick_pollution_status.sick_pollution_calibration_vals[i],&payload_buffer[k],2);
      _sick_pollution_status.sick_pollution_calibration_vals[i] =
	sick_lms_2xx_to_host_byte_order(_sick_pollution_status.sick_pollution_calibration_vals[i]);
    }

    /* Buffer the calibrating reference pollution values */
    for (unsigned int i = 0, k = 59; i < 4; i++, k+=2) {
      memcpy(&_sick_pollution_status.sick_reference_pollution_calibration_vals[i],&payload_buffer[k],2);
      _sick_pollution_status.sick_reference_pollution_calibration_vals[i] =
	sick_lms_2xx_to_host_byte_order(_sick_pollution_status.sick_reference_pollution_calibration_vals[i]);
    }

    /*
     * Extract the Sick LMS signal config 
     */
    
    /* Buffer the reference scale 1 value (Dark signal 100%) */
    memcpy(&_sick_signal_status.sick_reference_scale_1_dark_100,&payload_buffer[71],2);
    _sick_signal_status.sick_reference_scale_1_dark_100 =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_reference_scale_1_dark_100);

    /* Buffer the reference scale 2 value (Dark signal 100%) */
    memcpy(&_sick_signal_status.sick_reference_scale_2_dark_100,&payload_buffer[75],2);
    _sick_signal_status.sick_reference_scale_2_dark_100 =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_reference_scale_2_dark_100);

    /* Buffer the reference scale 1 value (Dark signal 66%) */
    memcpy(&_sick_signal_status.sick_reference_scale_1_dark_66,&payload_buffer[77],2);
    _sick_signal_status.sick_reference_scale_1_dark_66 =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_reference_scale_1_dark_66);

    /* Buffer the reference scale 2 value (Dark signal 100%) */
    memcpy(&_sick_signal_status.sick_reference_scale_2_dark_66,&payload_buffer[81],2);
    _sick_signal_status.sick_reference_scale_2_dark_66 =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_reference_scale_2_dark_66);

    /* Buffer the signal amplitude */
    memcpy(&_sick_signal_status.sick_signal_amplitude,&payload_buffer[83],2);
    _sick_signal_status.sick_signal_amplitude =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_signal_amplitude);

    /* Buffer the angle used for power measurement */
    memcpy(&_sick_signal_status.sick_current_angle,&payload_buffer[85],2);
    _sick_signal_status.sick_current_angle =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_current_angle);

    /* Buffer the peak threshold value */
    memcpy(&_sick_signal_status.sick_peak_threshold,&payload_buffer[87],2);
    _sick_signal_status.sick_peak_threshold =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_peak_threshold);
    
    /* Buffer the angle used for reference target power measurement */
    memcpy(&_sick_signal_status.sick_angle_of_measurement,&payload_buffer[89],2);
    _sick_signal_status.sick_angle_of_measurement =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_angle_of_measurement);

    /* Buffer the signal amplitude calibration value */
    memcpy(&_sick_signal_status.sick_signal_amplitude_calibration_val,&payload_buffer[91],2);
    _sick_signal_status.sick_signal_amplitude_calibration_val =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_signal_amplitude_calibration_val);

    /* Buffer the target value of stop threshold */
    memcpy(&_sick_signal_status.sick_stop_threshold_target_value,&payload_buffer[93],2);
    _sick_signal_status.sick_stop_threshold_target_value =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_stop_threshold_target_value);
    
    /* Buffer the target value of peak threshold */
    memcpy(&_sick_signal_status.sick_peak_threshold_target_value,&payload_buffer[95],2);
    _sick_signal_status.sick_peak_threshold_target_value =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_peak_threshold_target_value);
    
    /* Buffer the actual value of stop threshold */
    memcpy(&_sick_signal_status.sick_stop_threshold_actual_value,&payload_buffer[97],2);
    _sick_signal_status.sick_stop_threshold_actual_value =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_stop_threshold_actual_value);

    /* Buffer the actual value of peak threshold */
    memcpy(&_sick_signal_status.sick_peak_threshold_actual_value,&payload_buffer[99],2);
    _sick_signal_status.sick_peak_threshold_actual_value =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_peak_threshold_actual_value);

    /* Buffer reference target "single measured values" */
    memcpy(&_sick_signal_status.sick_reference_target_single_measured_vals,&payload_buffer[103],2);
    _sick_signal_status.sick_reference_target_single_measured_vals =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_reference_target_single_measured_vals);
  
    /* Buffer reference target "mean measured values" */
    memcpy(&_sick_signal_status.sick_reference_target_mean_measured_vals,&payload_buffer[105],2);
    _sick_signal_status.sick_reference_target_mean_measured_vals =
      sick_lms_2xx_to_host_byte_order(_sick_signal_status.sick_reference_target_mean_measured_vals);


    /*
     * Extract the Sick LMS field config
     */

    /* Buffer the offset for multiple evaluations of field set 2 */
    _sick_field_status.sick_multiple_evaluation_offset_field_2 = payload_buffer[114];

    /* Buffer the evaluation number */
    _sick_field_status.sick_field_evaluation_number = payload_buffer[118];

    /* Buffer the active field set number */
    _sick_field_status.sick_field_set_number = payload_buffer[121];


    /*
     * Extract the Sick LMS baud config
     */
    
    /* Buffer the permanent baud rate flag */
    _sick_baud_status.sick_permanent_baud_rate = payload_buffer[119];
    
    /* Buffer the baud rate of the device */
    memcpy(&_sick_baud_status.sick_baud_rate,&payload_buffer[116],2);
    _sick_baud_status.sick_baud_rate =
      sick_lms_2xx_to_host_byte_order(_sick_baud_status.sick_baud_rate);

    /* Buffer calibration value 1 for counter 0 */
    //memcpy(&_sick_status_data.sick_calibration_counter_0_value_1,&payload_buffer[131],4);
    //_sick_status_data.sick_calibration_counter_0_value_1 =
    //  sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_calibration_counter_0_value_1);

    /* Buffer calibration value 2 for counter 0 */
    //memcpy(&_sick_status_data.sick_calibration_counter_0_value_2,&payload_buffer[135],4);
    //_sick_status_data.sick_calibration_counter_0_value_2 =
    //  sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_calibration_counter_0_value_2);

    /* Buffer calibration value 1 for counter 1 */
    //memcpy(&_sick_status_data.sick_calibration_counter_1_value_1,&payload_buffer[139],4);
    //_sick_status_data.sick_calibration_counter_1_value_1 =
    //  sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_calibration_counter_1_value_1);

    /* Buffer calibration value 2 for counter 1 */
    //memcpy(&_sick_status_data.sick_calibration_counter_1_value_2,&payload_buffer[143],4);
    //_sick_status_data.sick_calibration_counter_1_value_2 =
    //  sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_calibration_counter_1_value_2);

    /* Buffer M0 value counter 0 */
    //memcpy(&_sick_status_data.sick_counter_0_M0,&payload_buffer[147],2);
    //_sick_status_data.sick_counter_0_M0 = sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_counter_0_M0);

    /* Buffer M0 value counter 1 */
    //memcpy(&_sick_status_data.sick_counter_1_M0,&payload_buffer[149],2);
    //_sick_status_data.sick_counter_1_M0 = sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_counter_1_M0);

    /* Buffer calibration interval */
    //memcpy(&_sick_status_data.sick_calibration_interval,&payload_buffer[151],2);
    //_sick_status_data.sick_calibration_interval = sick_lms_2xx_to_host_byte_order(_sick_status_data.sick_calibration_interval);
  
  }

  /**
   * \brief Sets the device to installation mode
   */
  void SickLMS2xx::_setSickOpModeInstallation( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {
    
    /* Assign the password for entering installation mode */
    uint8_t sick_password[9] = DEFAULT_SICK_LMS_2XX_SICK_PASSWORD;

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_INSTALLATION) {

      try {

	/* Attempt to switch modes! */	
	_switchSickOperatingMode(SICK_OP_MODE_INSTALLATION,sick_password);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeInstallation: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_INSTALLATION;

      /* Reset these parameters */
      _sick_mean_value_sample_size = _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;

    }

  }

  /**
   * \brief Sets the device to diagnostic mode
   */
  void SickLMS2xx::_setSickOpModeDiagnostic( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_DIAGNOSTIC) {

      std::cout << "\tAttempting to enter diagnostic mode..." << std::endl;
      
      try {

	/* Attempt to switch modes! */	
	_switchSickOperatingMode(SICK_OP_MODE_DIAGNOSTIC);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeInstallation: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_DIAGNOSTIC;

      /* Reset these parameters */
      _sick_mean_value_sample_size = _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;

      std::cout << "Success!" << std::endl;
      
    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to send values only upon request
   */
  void SickLMS2xx::_setSickOpModeMonitorRequestValues( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_REQUEST_VALUES) {

      try {

	/* Attempt to switch operating mode */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_REQUEST_VALUES);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeMonitorRequestValues: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_REQUEST_VALUES;

      /* Reset these parameters */
      _sick_mean_value_sample_size = _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;

    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to stream measured values
   */
  void SickLMS2xx::_setSickOpModeMonitorStreamValues( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_VALUES) {

      std::cout << "\tRequesting measured value data stream..." << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_VALUES);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeMonitorStreamValues: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_VALUES;

      /* Reset these parameters */
      _sick_mean_value_sample_size = _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;

      std::cout << "\t\tData stream started!" << std::endl;
      
    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to stream both range and reflectivity values
   */
  void SickLMS2xx::_setSickOpModeMonitorStreamRangeAndReflectivity( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* A sanity check to make sure that the command is supported */
    if (_sick_type != SICK_LMS_TYPE_211_S14 && _sick_type != SICK_LMS_TYPE_221_S14 && _sick_type != SICK_LMS_TYPE_291_S14) {
      throw SickConfigException("SickLMS2xx::_setSickOpModeMonitorStreamRangeAndReflectivity: Mode not supported by this model!");
    }
    
    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT) {

      /* Define the parameters */
      uint8_t mode_params[4] = {0x01,0x00,0xB5,0x00}; //1 to 181

      std::cout << "\tRequesting range & reflectivity data stream..." << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT,mode_params);

      }
      
      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeStreamRangeAndReflectivity: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT;

      /* Reset these parameters */
      _sick_mean_value_sample_size = _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;

      std::cout << "\t\tData stream started!" << std::endl;
      
    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to start sending partial scans
   */
  void SickLMS2xx::_setSickOpModeMonitorStreamValuesFromPartialScan( )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN) {

      std::cout << "\tRequesting partial scan data stream..." << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeStreamValuesFromPartialScan: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN;

      /* Reset these parameters */
      _sick_mean_value_sample_size = _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;

      std::cout << "\t\tData stream started!" << std::endl;
      
    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to send mean measured values
   */
  void SickLMS2xx::_setSickOpModeMonitorStreamMeanValues( const uint8_t sample_size )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES ||
	_sick_mean_value_sample_size != sample_size) {

      /* Make sure the sample size is legitimate */
      if(sample_size < 2 || sample_size > 250) {
	throw SickConfigException("SickLMS2xx::_setSickOpModeMonitorStreamMeanValues: Invalid sample size!");
      }

      std::cout << "\tRequesting mean value data stream (sample size = " << (int)sample_size << ")..." << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES,&sample_size);

      }

      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeStreamRangeFromPartialScan: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES;

      /* Buffer the current sample size! */
      _sick_mean_value_sample_size = sample_size;

      /* Reset these parameters */
      _sick_values_subrange_start_index = _sick_values_subrange_stop_index = 0;
      
      std::cout << "\t\tData stream started!" << std::endl;
      
    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to send a measured value subrange
   * \param subrange_start_index The starting index of the desired subrange
   * \param subrange_stop_index The stopping index of the desired subrange
   */
  void SickLMS2xx::_setSickOpModeMonitorStreamValuesSubrange( const uint16_t subrange_start_index, const uint16_t subrange_stop_index )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE ||
	_sick_values_subrange_start_index != subrange_start_index ||
	_sick_values_subrange_stop_index != subrange_stop_index ) {

      /* Compute the maximum subregion bound */
      unsigned int max_subrange_stop_index = (unsigned int)((_sick_operating_status.sick_scan_angle*100)/_sick_operating_status.sick_scan_resolution + 1) ;
      
      /* Ensure the subregion is properly defined for the given variant */
      if(subrange_start_index > subrange_stop_index || subrange_start_index == 0 || subrange_stop_index > max_subrange_stop_index) {
	throw SickConfigException("SickLMS2xx::_setSickOpMonitorStreamValuesSubrange: Invalid subregion bounds!");
      }
      
      /* Setup a few buffers */
      uint8_t mode_params[4] = {0};
      uint16_t temp_buffer = 0;

      /* Assign the subrange start index */
      temp_buffer = host_to_sick_lms_2xx_byte_order(subrange_start_index);
      memcpy(mode_params,&temp_buffer,2);

      /* Assign the subrange stop index */
      temp_buffer = host_to_sick_lms_2xx_byte_order(subrange_stop_index);
      memcpy(&mode_params[2],&temp_buffer,2);
      
      std::cout << "\tRequesting measured value stream... (subrange = [" << subrange_start_index << "," << subrange_stop_index << "])" << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE,mode_params);

      }
      
      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeInstallation: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE;

      /* Reset this parameter */
      _sick_mean_value_sample_size = 0;

      /* Buffer the starting/stopping indices */
      _sick_values_subrange_start_index = subrange_start_index;
      _sick_values_subrange_stop_index = subrange_stop_index;
      
      std::cout << "\t\tData stream started!" << std::endl;

    }

  }

  /**
   * \brief Sets the device to monitor mode and tells it to send a mean value subrange
   * \param sample_size The number of scans to consider in computing the mean measured values
   * \param subrange_start_index The starting index of the desired subrange
   * \param subrange_stop_index The stopping index of the desired subrange
   */
  void SickLMS2xx::_setSickOpModeMonitorStreamMeanValuesSubrange( const uint16_t sample_size, const uint16_t subrange_start_index, const uint16_t subrange_stop_index )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    /* Check if mode should be changed */
    if (_sick_operating_status.sick_operating_mode != SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE ||
	_sick_values_subrange_start_index != subrange_start_index ||
	_sick_values_subrange_stop_index != subrange_stop_index ||
	_sick_mean_value_sample_size != sample_size ) {

      /* Make sure the sample size is legit */
      if(sample_size < 2 || sample_size > 250) {
	throw SickConfigException("SickLMS2xx::_setSickOpModeMonitorStreamMeanValuesSubrange: Invalid sample size!");
      }
      
      /* Compute the maximum subregion bound */
      unsigned int max_subrange_stop_index = (unsigned int)((_sick_operating_status.sick_scan_angle*100)/_sick_operating_status.sick_scan_resolution + 1) ;
      
      /* Ensure the subregion is properly defined for the given variant */
      if(subrange_start_index > subrange_stop_index || subrange_start_index == 0 || subrange_stop_index > max_subrange_stop_index) {
	throw SickConfigException("SickLMS2xx::_setSickOpMonitorStreamMeanValuesSubrange: Invalid subregion bounds!");
      }
      
      /* Setup a few buffers */
      uint8_t mode_params[5] = {0};
      uint16_t temp_buffer = 0;

      /* Assign the sample size */
      mode_params[0] = sample_size;

      /* Assign the subrange start index */
      temp_buffer = host_to_sick_lms_2xx_byte_order(subrange_start_index);
      memcpy(&mode_params[1],&temp_buffer,2);

      /* Assign the subrange stop index */
      temp_buffer = host_to_sick_lms_2xx_byte_order(subrange_stop_index);
      memcpy(&mode_params[3],&temp_buffer,2);
      
      std::cout << "\tRequesting mean value stream... (subrange = [" << subrange_start_index << "," << subrange_stop_index << "])" << std::endl;
      
      try {

	/* Attempt to switch modes */
	_switchSickOperatingMode(SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE,mode_params);

      }
      
      /* Catch any config exceptions */
      catch(SickConfigException &sick_config_exception) {
	std::cerr << sick_config_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any timeout exceptions */
      catch(SickTimeoutException &sick_timeout_exception) {
	std::cerr << sick_timeout_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any I/O exceptions */
      catch(SickIOException &sick_io_exception) {
	std::cerr << sick_io_exception.what() << std::endl;
	throw;
      }
      
      /* Catch any thread exceptions */
      catch(SickThreadException &sick_thread_exception) {
	std::cerr << sick_thread_exception.what() << std::endl;
	throw;
      }
      
      /* Catch anything else */
      catch(...) {
	std::cerr << "SickLMS2xx::_setSickOpModeInstallation: Unknown exception!!!" << std::endl;
	throw;
      } 
      
      /* Assign the new operating mode */
      _sick_operating_status.sick_operating_mode = SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE;

      /* Buffer the sample size */
      _sick_mean_value_sample_size = sample_size;

      /* Buffer the starting/stopping indices */
      _sick_values_subrange_start_index = subrange_start_index;
      _sick_values_subrange_stop_index = subrange_stop_index;
      
      std::cout << "\t\tData stream started!" << std::endl;

    }

  }
  
  /**
   * \brief Attempts to switch the operating mode of the Sick LMS 2xx
   * \param sick_mode The desired operating mode
   * \param mode_params Additional parameters required to set the new operating mode
   */
  void SickLMS2xx::_switchSickOperatingMode( const uint8_t sick_mode, const uint8_t * const mode_params )
    throw( SickConfigException, SickIOException, SickThreadException, SickTimeoutException) {

    SickLMS2xxMessage message,response;

    uint8_t payload_buffer[SickLMS2xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};    
    uint16_t num_partial_scans = 0;

    /* Construct the correct switch mode packet */
    payload_buffer[0] = 0x20;
    payload_buffer[1] = sick_mode;

    switch(sick_mode) {
   
    case SICK_OP_MODE_INSTALLATION:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      memcpy(&payload_buffer[2],mode_params,8); //Copy password
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,10);
      break;

    case SICK_OP_MODE_DIAGNOSTIC:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_REQUEST_VALUES:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      payload_buffer[2] = *mode_params;
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,3);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      memcpy(&payload_buffer[2],mode_params,2);       //Begin range
      memcpy(&payload_buffer[4],&mode_params[2],2);   //End range
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,6);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      payload_buffer[2] = mode_params[0];             //Sample size 
      memcpy(&payload_buffer[3],&mode_params[1],2);   //Begin mean range
      memcpy(&payload_buffer[5],&mode_params[3],2);   //End mean range
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,7);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      memcpy(&payload_buffer[2],mode_params,2);       //Start
      memcpy(&payload_buffer[4],&mode_params[2],2);   //End
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,6);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }

      /* Get the number of partial scans (between 1 and 5) */
      memcpy(&num_partial_scans,mode_params,2);

      /* Setup the command packet */
      memcpy(&payload_buffer[2],mode_params,num_partial_scans*4+2);
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,num_partial_scans*4+4);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }
    
      /* Get the number of partial scans (between 1 and 5) */
      memcpy(&num_partial_scans,mode_params,2);
    
      /* Setup the command packet */
      memcpy(&payload_buffer[2],mode_params,num_partial_scans*4+2);    
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,num_partial_scans*4+4);
      break;

    case SICK_OP_MODE_MONITOR_NAVIGATION:
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,2);
      break;

    case SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT:

      /* Make sure the params are defined */
      if(mode_params == NULL) {
	throw SickConfigException("SickLMS2xx::_switchSickOperatingMode - Requested mode requires parameters!");
      }
      
      memcpy(&payload_buffer[2],mode_params,2);       //Start
      memcpy(&payload_buffer[4],&mode_params[2],2);   //End
      message.BuildMessage(DEFAULT_SICK_LMS_2XX_SICK_ADDRESS,payload_buffer,6);
      break;

    case SICK_OP_MODE_UNKNOWN:
      //Let this case go straight to default
    
    default:
      throw SickConfigException("SickLMS2xx::_switchSickOperatingMode: Unrecognized operating mode!");
    }

    try {

      /* Attempt to send the message and get the reply */
      _sendMessageAndGetReply(message,response,DEFAULT_SICK_LMS_2XX_SICK_SWITCH_MODE_TIMEOUT,DEFAULT_SICK_LMS_2XX_NUM_TRIES);
      
    }

    /* Catch any timeout exceptions */
    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Catch any I/O exceptions */
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* Catch any thread exceptions */
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    /* Catch anything else */
    catch(...) {
      std::cerr << "SickLMS2xx::_switchSickOperatingMode: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer */
    memset(payload_buffer,0,sizeof(payload_buffer));
  
    /* Obtain the response payload */
    response.GetPayload(payload_buffer);

    /* Make sure the reply was expected */
    if(payload_buffer[1] != 0x00) {
      throw SickConfigException("SickLMS2xx::_switchSickOperatingMode: configuration request failed!");
    }

  }

  /**
   * \brief Parses a byte sequence into a scan profile corresponding to message B0
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_scan_profile The returned scan profile for the current round of measurements
   */
  void SickLMS2xx::_parseSickScanProfileB0( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_b0_t &sick_scan_profile ) const {

    /* Read block A, the number of measurments */
    sick_scan_profile.sick_num_measurements = src_buffer[0] + 256*(src_buffer[1] & 0x03);

    /* Check whether this is a partial scan */
    sick_scan_profile.sick_partial_scan_index = ((src_buffer[1] & 0x18) >> 3);

    /* Extract the measurements and Field values (if there are any) */
    _extractSickMeasurementValues(&src_buffer[2],
				  sick_scan_profile.sick_num_measurements,
				  sick_scan_profile.sick_measurements,
				  sick_scan_profile.sick_field_a_values,
				  sick_scan_profile.sick_field_b_values,
				  sick_scan_profile.sick_field_c_values);
    
    /* If the Sick is pulling real-time indices then pull them too */
    unsigned int data_offset = 2 + 2*sick_scan_profile.sick_num_measurements;
    if (_returningRealTimeIndices()) {
      sick_scan_profile.sick_real_time_scan_index = src_buffer[data_offset];
      data_offset++;
    }

    /* Buffer the Sick telegram index */
    sick_scan_profile.sick_telegram_index = src_buffer[data_offset];
    
  }

  /**
   * \brief Parses a byte sequence into a scan profile corresponding to message B6
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_scan_profile The returned scan profile for the current round of mean measurements
   */
  void SickLMS2xx::_parseSickScanProfileB6( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_b6_t &sick_scan_profile ) const {

    /* Read Block A, the sample size used in computing the mean return */
    sick_scan_profile.sick_sample_size = src_buffer[0];

    /* Read Block B, the number of measured values sent */
    sick_scan_profile.sick_num_measurements = src_buffer[1] + 256*(src_buffer[2] & 0x03);

    /* Read Block C, extract the range measurements and Field values (if there are any) */
    _extractSickMeasurementValues(&src_buffer[3],
				  sick_scan_profile.sick_num_measurements,
				  sick_scan_profile.sick_measurements);
    
    /* Read Block D, if the Sick is pulling real-time indices then pull them too */
    unsigned int data_offset = 3 + 2*sick_scan_profile.sick_num_measurements;
    if (_returningRealTimeIndices()) {
      sick_scan_profile.sick_real_time_scan_index = src_buffer[data_offset];
      data_offset++;
    }

    /* Read Block E, buffer the Sick telegram index */
    sick_scan_profile.sick_telegram_index = src_buffer[data_offset];
    
  }
  
  /**
   * \brief Parses a byte sequence into a scan profile corresponding to message B7
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_scan_profile The returned scan profile for the current round of mean measurements
   */
  void SickLMS2xx::_parseSickScanProfileB7( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_b7_t &sick_scan_profile ) const {

    /* Read Block A, Sick LMS measured value subrange start index */
    sick_scan_profile.sick_subrange_start_index = src_buffer[0] + 256*src_buffer[1];

    /* Read Block B, Sick LMS measured value subrange stop index */
    sick_scan_profile.sick_subrange_stop_index = src_buffer[2] + 256*src_buffer[3];
    
    /* Read block C, the number of measurements */
    sick_scan_profile.sick_num_measurements = src_buffer[4] + 256*(src_buffer[5] & 0x03);

    /* Acquire the partial scan index (also in Block C) */
    sick_scan_profile.sick_partial_scan_index = ((src_buffer[5] & 0x18) >> 3);
    
    /* Read Block D, extract the range measurements and Field values (if there are any) */
    _extractSickMeasurementValues(&src_buffer[6],
				  sick_scan_profile.sick_num_measurements,
				  sick_scan_profile.sick_measurements,
				  sick_scan_profile.sick_field_a_values,
				  sick_scan_profile.sick_field_b_values,
				  sick_scan_profile.sick_field_c_values);
    
    /* Read Block E, if the Sick is pulling real-time indices then pull them too */
    unsigned int data_offset = 6 + 2*sick_scan_profile.sick_num_measurements;
    if (_returningRealTimeIndices()) {
      sick_scan_profile.sick_real_time_scan_index = src_buffer[data_offset];
      data_offset++;
    }

    /* Read Block F, buffer the Sick telegram index */
    sick_scan_profile.sick_telegram_index = src_buffer[data_offset];
    
  }

  /**
   * \brief Parses a byte sequence into a scan profile corresponding to message B6
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_scan_profile The returned scan profile for the current round of mean measurements
   */
  void SickLMS2xx::_parseSickScanProfileBF( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_bf_t &sick_scan_profile ) const {

    /* Read Block A, the sample size used in computing the mean return */
    sick_scan_profile.sick_sample_size = src_buffer[0];

    /* Read Block B, Sick LMS measured value subrange start index */
    sick_scan_profile.sick_subrange_start_index = src_buffer[1] + 256*src_buffer[2];

    /* Read Block C, Sick LMS measured value subrange stop index */
    sick_scan_profile.sick_subrange_stop_index = src_buffer[3] + 256*src_buffer[4];
    
    /* Read Block D, the number of measured values sent */
    sick_scan_profile.sick_num_measurements = src_buffer[5] + 256*(src_buffer[6] & 0x3F);

    /* Read Block E, extract the mean measurements */
    _extractSickMeasurementValues(&src_buffer[7],
				  sick_scan_profile.sick_num_measurements,
				  sick_scan_profile.sick_measurements);
    
    /* Read Block D, if the Sick is pulling real-time indices then pull them too */
    unsigned int data_offset = 7 + 2*sick_scan_profile.sick_num_measurements;
    if (_returningRealTimeIndices()) {
      sick_scan_profile.sick_real_time_scan_index = src_buffer[data_offset];
      data_offset++;
    }

    /* Read Block E, buffer the Sick telegram index */
    sick_scan_profile.sick_telegram_index = src_buffer[data_offset];
    
  }
  
  /**
   * \brief Parses a byte sequence into a scan profile corresponding to message C4
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_scan_profile The returned scan profile for the current round of measurements
   */
  void SickLMS2xx::_parseSickScanProfileC4( const uint8_t * const src_buffer, sick_lms_2xx_scan_profile_c4_t &sick_scan_profile ) const {

    /* Read block A - the number of range measurments.  We need the low two bits
     * of the most significant byte. */
    sick_scan_profile.sick_num_range_measurements = src_buffer[0] + 256*(src_buffer[1] & 0x03);

    /* Read Block B - Extract the range measurements and Field values (if there are any) */
    _extractSickMeasurementValues(&src_buffer[2],
				  sick_scan_profile.sick_num_range_measurements,
				  sick_scan_profile.sick_range_measurements,
				  sick_scan_profile.sick_field_a_values,
				  sick_scan_profile.sick_field_b_values,
				  sick_scan_profile.sick_field_c_values);
    
    /* Read Block D - Extract the number of range measurements */
    unsigned int data_offset = 2 + 2*sick_scan_profile.sick_num_range_measurements;
    sick_scan_profile.sick_num_reflect_measurements = src_buffer[data_offset] + 256*(src_buffer[data_offset+1] & 0x03);
    data_offset += 2;
    
    /* Read Block E - Extract the reflectivity value subrange start index */
    sick_scan_profile.sick_reflect_subrange_start_index = src_buffer[data_offset] + 256*src_buffer[data_offset+1];
    data_offset += 2;

    /* Read Block F - Extract the reflectivity value subrange stop index */
    sick_scan_profile.sick_reflect_subrange_stop_index = src_buffer[data_offset] + 256*src_buffer[data_offset+1];
    data_offset += 2;

    /* Read blocks G...H. Mask out the reflectivity values and store in provided buffer. */
    for(unsigned int i=0; i < sick_scan_profile.sick_num_reflect_measurements; i++,data_offset++) {
      sick_scan_profile.sick_reflect_measurements[i] = src_buffer[data_offset];
    }
    
    /* Read Block I - real-time scan indices */
    if (_returningRealTimeIndices()) {
      sick_scan_profile.sick_real_time_scan_index = src_buffer[data_offset];
      data_offset++;
    }

    /* Read Block J - the telegram scan index */
    sick_scan_profile.sick_telegram_index = src_buffer[data_offset];
    
  }
  
  /**
   * \brief Parses a byte sequence into a Sick config structure
   * \param *src_buffer The byte sequence to be parsed
   * \param &sick_device_config The device configuration
   */
  void SickLMS2xx::_parseSickConfigProfile( const uint8_t * const src_buffer, sick_lms_2xx_device_config_t &sick_device_config ) const {

    /* Buffer Block A */
    memcpy(&sick_device_config.sick_blanking,&src_buffer[0],2);
    sick_device_config.sick_blanking = sick_lms_2xx_to_host_byte_order(sick_device_config.sick_blanking);
    
    /* Buffer Block B */
    sick_device_config.sick_peak_threshold = src_buffer[3]; // NOTE: This value represent sensitivity for LMS 211/221/291
    sick_device_config.sick_stop_threshold = src_buffer[2]; // NOTE: This value will be 0 for LMS 211/221/291
    
    /* Buffer Block C */
    sick_device_config.sick_availability_level = src_buffer[4];
    
    /* Buffer Block D */
    sick_device_config.sick_measuring_mode = src_buffer[5];
    
    /* Buffer Block E */
    sick_device_config.sick_measuring_units = src_buffer[6];
    
    /* Buffer Block F */
    sick_device_config.sick_temporary_field = src_buffer[7];
    
    /* Buffer Block G */
    sick_device_config.sick_subtractive_fields = src_buffer[8];
    
    /* Buffer Block H */
    sick_device_config.sick_multiple_evaluation = src_buffer[9];
    
    /* Buffer Block I */
    sick_device_config.sick_restart = src_buffer[10];
    
    /* Buffer Block J */
    sick_device_config.sick_restart_time = src_buffer[11];
    
    /* Buffer Block K */
    sick_device_config.sick_multiple_evaluation_suppressed_objects = src_buffer[12];
    
    /* Buffer Block L */
    sick_device_config.sick_contour_a_reference = src_buffer[13];
    
    /* Buffer Block M */
    sick_device_config.sick_contour_a_positive_tolerance_band = src_buffer[14];
    
    /* Buffer Block N */
    sick_device_config.sick_contour_a_negative_tolerance_band = src_buffer[15];
    
    /* Buffer Block O */
    sick_device_config.sick_contour_a_start_angle = src_buffer[16];
    
    /* Buffer Block P */
    sick_device_config.sick_contour_a_stop_angle = src_buffer[17];
    
    /* Buffer Block Q */
    sick_device_config.sick_contour_b_reference = src_buffer[18];
    
    /* Buffer Block R */
    sick_device_config.sick_contour_b_positive_tolerance_band = src_buffer[19];
    
    /* Buffer Block S */
    sick_device_config.sick_contour_b_negative_tolerance_band = src_buffer[20];
    
    /* Buffer Block T */
    sick_device_config.sick_contour_b_start_angle = src_buffer[21];
    
    /* Buffer Block U */
    sick_device_config.sick_contour_b_stop_angle = src_buffer[22];
    
    /* Buffer Block V */
    sick_device_config.sick_contour_c_reference = src_buffer[23];
    
    /* Buffer Block W */
    sick_device_config.sick_contour_c_positive_tolerance_band = src_buffer[24];

    /* Buffer Block X */
    sick_device_config.sick_contour_c_negative_tolerance_band = src_buffer[25];
    
    /* Buffer Block Y */
    sick_device_config.sick_contour_c_start_angle = src_buffer[26];
    
    /* Buffer Block Z */
    sick_device_config.sick_contour_c_stop_angle = src_buffer[27];
    
    /* Buffer Block A1 */
    sick_device_config.sick_pixel_oriented_evaluation = src_buffer[28];
    
    /* Buffer Block A2 */
    sick_device_config.sick_single_measured_value_evaluation_mode = src_buffer[29];
    
    /* Buffer Block A3 */
    memcpy(&sick_device_config.sick_fields_b_c_restart_times,&src_buffer[30],2);
    sick_device_config.sick_fields_b_c_restart_times =
      sick_lms_2xx_to_host_byte_order(sick_device_config.sick_fields_b_c_restart_times);
    
    /* Buffer Block A4 */
    memcpy(&sick_device_config.sick_dazzling_multiple_evaluation,&src_buffer[32],2);
    sick_device_config.sick_dazzling_multiple_evaluation =
      sick_lms_2xx_to_host_byte_order(sick_device_config.sick_dazzling_multiple_evaluation);
    
  }

  /**
   * \brief Extracts the measured values (w/ flags) that were returned by the device.
   * \param *byte_sequence The byte sequence holding the current measured values
   * \param num_measurements The number of measurements given in the byte sequence
   * \param *measured_values A buffer to hold the extracted measured values
   * \param *field_a_values Stores the Field A values associated with the given measurements (Default: NULL => Not wanted)
   * \param *field_b_values Stores the Field B values associated with the given measurements (Default: NULL => Not wanted)
   * \param *field_c_values Stores the Field C values associated with the given measurements (Default: NULL => Not wanted)
   */
  void SickLMS2xx::_extractSickMeasurementValues( const uint8_t * const byte_sequence, const uint16_t num_measurements, uint16_t * const measured_values,
					       uint8_t * const field_a_values, uint8_t * const field_b_values, uint8_t * const field_c_values ) const {

    /* Parse the byte sequence and fill the return buffer with range measurements... */   
    switch(_sick_device_config.sick_measuring_mode) {
    case SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE:
      {

	/* Extract the range and Field values */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x1F);

	  if(field_a_values) {  
	    field_a_values[i] = byte_sequence[i*2+1] & 0x20;
	  }
	  
	  if(field_b_values) {
	    field_b_values[i] = byte_sequence[i*2+1] & 0x40;
	  }
	  
	  if(field_c_values) {
	    field_c_values[i] = byte_sequence[i*2+1] & 0x80;
	  }
	  
	}
	
	break;
      }
    case SICK_MS_MODE_8_OR_80_REFLECTOR:
      {
	
	/* Extract the range and Field A */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x1F);
	  
	  if(field_a_values) {
	    field_a_values[i] = byte_sequence[i*2+1] & 0xE0;
	  }
	  
	}
	
	break;
      }     
    case SICK_MS_MODE_8_OR_80_FA_FB_FC:
      {
	
	/* Extract the range and Fields A,B and C */	
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x1F);
	  
	  if(field_a_values) {
	    field_a_values[i] = byte_sequence[i*2+1] & 0x20;
	  }
	  
	  if(field_b_values) {
	    field_b_values[i] = byte_sequence[i*2+1] & 0x40;
	  }
	  
	  if(field_c_values) {
	    field_c_values[i] = byte_sequence[i*2+1] & 0x80;
	  }
	  
	}
	
	break;
      }
    case SICK_MS_MODE_16_REFLECTOR:
      {

	/* Extract the range and reflector values */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x3F);

	  if (field_a_values) {
	    field_a_values[i] = byte_sequence[i*2+1] & 0xC0;
	  }
	  
	}
	
	break;
      }
    case SICK_MS_MODE_16_FA_FB:
      {

	/* Extract the range and Fields A and B values */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x3F);

	  if(field_a_values) {
	    field_a_values[i] = byte_sequence[i*2+1] & 0x40;
	  }

	  if(field_b_values) {
	    field_b_values[i] = byte_sequence[i*2+1] & 0x80;
	  }

	}
	
	break;
      }
    case SICK_MS_MODE_32_REFLECTOR:
      {

	/* Extract the range and reflector values */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x7F);

	  if(field_a_values) {
	    field_a_values[i] = byte_sequence[i*2+1] & 0x80;
	  }
	  
	}
	
	break;
      }
    case SICK_MS_MODE_32_FA:
      {

	/* Extract the range and Field A values */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1] & 0x7F);

	  if(field_a_values) {
	    field_a_values[i] = byte_sequence[i*2+1] & 0x80;
	  }
	  
	}
	
	break;
      }
    case SICK_MS_MODE_32_IMMEDIATE:
      {
	
	/* Extract the range measurements (no flags for this mode */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1]);
	}
	
	break;
      }
    case SICK_MS_MODE_REFLECTIVITY:
      {

	/* Extract the reflectivity values */
	for(unsigned int i = 0; i < num_measurements; i++) {
	  measured_values[i] = byte_sequence[i*2] + 256*(byte_sequence[i*2+1]);
	}
	
	break;
      }      
    default:      
      break;
    }
    
  }
  
  /**
   * \brief Indicates whether the given measuring units are valid/defined
   * \param sick_units The units in question
   */ 
  bool SickLMS2xx::_validSickMeasuringUnits( const sick_lms_2xx_measuring_units_t sick_units ) const {

    /* Check the given units value */
    if (sick_units != SICK_MEASURING_UNITS_CM && sick_units != SICK_MEASURING_UNITS_MM) {
      return false;
    }

    /* Valid */
    return true;
  }

  /**
   * \brief Indicates whether the Sick is an LMS 200
   * \return True if the device is a Sick LMS 200, False otherwise
   */ 
  bool SickLMS2xx::_isSickLMS200( ) const {

    /* Check the given Sick type value */
    switch(_sick_type) {
    case SICK_LMS_TYPE_200_30106:
      return true;
    default:
      return false;
    }

  }

  /**
   * \brief Indicates whether the Sick is an LMS 211
   * \return True if the device is a Sick LMS 211, False otherwise
   */ 
  bool SickLMS2xx::_isSickLMS211( ) const {

    /* Check the given Sick type value */
    switch(_sick_type) {
    case SICK_LMS_TYPE_211_30106:
      return true;
    case SICK_LMS_TYPE_211_30206:
      return true;
    case SICK_LMS_TYPE_211_S07:
      return true;
    case SICK_LMS_TYPE_211_S14:
      return true;
    case SICK_LMS_TYPE_211_S15:
      return true;
    case SICK_LMS_TYPE_211_S19:
      return true;
    case SICK_LMS_TYPE_211_S20:
      return true;
    default:
      return false;
    }

  }

  /**
   * \brief Indicates whether the Sick is an LMS 220
   * \return True if the device is a Sick LMS 220, False otherwise
   */ 
  bool SickLMS2xx::_isSickLMS220( ) const {

    /* Check the given Sick type value */
    switch(_sick_type) {
    case SICK_LMS_TYPE_220_30106:
      return true;
    default:
      return false;
    }

  }

  /**
   * \brief Indicates whether the Sick is an LMS 221
   * \return True if the device is a Sick LMS 221, False otherwise
   */ 
  bool SickLMS2xx::_isSickLMS221( ) const {

    /* Check the given Sick type value */
    switch(_sick_type) {
    case SICK_LMS_TYPE_221_30106:
      return true;
    case SICK_LMS_TYPE_221_30206:
      return true;
    case SICK_LMS_TYPE_221_S07:
      return true;
    case SICK_LMS_TYPE_221_S14:
      return true;
    case SICK_LMS_TYPE_221_S15:
      return true;
    case SICK_LMS_TYPE_221_S16:
      return true;
    case SICK_LMS_TYPE_221_S19:
      return true;
    case SICK_LMS_TYPE_221_S20:
      return true;
    default:
      return false;
    }

  }

  /**
   * \brief Indicates whether the Sick is an LMS 291
   * \return True if the device is a Sick LMS 291, False otherwise
   */ 
  bool SickLMS2xx::_isSickLMS291( ) const {

    /* Check the given Sick type value */
    switch(_sick_type) {
    case SICK_LMS_TYPE_291_S05:
      return true;
    case SICK_LMS_TYPE_291_S14:
      return true;
    case SICK_LMS_TYPE_291_S15:
      return true;
    default:
      return false;
    }

  }

  /**
   * \brief Indicates whether the Sick type is unknown
   * \return True if the device is unknown, False otherwise
   */ 
  bool SickLMS2xx::_isSickUnknown( ) const {
    return _sick_type == SICK_LMS_TYPE_UNKNOWN;
  }
    
  /**
   * \brief Indicates whether the given scan angle is defined
   * \param sick_scan_angle The scan angle in question
   */ 
  bool SickLMS2xx::_validSickScanAngle( const sick_lms_2xx_scan_angle_t sick_scan_angle ) const {

    /* Check the given Sick scan angle */
    if (sick_scan_angle != SICK_SCAN_ANGLE_90 &&
	sick_scan_angle != SICK_SCAN_ANGLE_100 &&
	sick_scan_angle != SICK_SCAN_ANGLE_180 ) {
      
      return false;
    }

    /* Valid */
    return true;
  }

  /**
   * \brief Indicates whether the given scan resolution is defined
   * \param sick_scan_resolution The scan resolution in question
   */ 
  bool SickLMS2xx::_validSickScanResolution( const sick_lms_2xx_scan_resolution_t sick_scan_resolution ) const {

    /* Check the given Sick scan resolution value */
    if (sick_scan_resolution != SICK_SCAN_RESOLUTION_25 &&
	sick_scan_resolution != SICK_SCAN_RESOLUTION_50 &&
	sick_scan_resolution != SICK_SCAN_RESOLUTION_100 ) {
      
      return false;
    }

    /* Valid */
    return true;
  }
  
  /**
   * \brief Indicates whether the given sensitivity is defined
   * \param sick_sensitivity The sick sensitivity in question
   */ 
  bool SickLMS2xx::_validSickSensitivity( const sick_lms_2xx_sensitivity_t sick_sensitivity ) const {

    /* Check the given Sick sensitivity value */
    if (sick_sensitivity != SICK_SENSITIVITY_STANDARD &&
	sick_sensitivity != SICK_SENSITIVITY_MEDIUM &&
	sick_sensitivity != SICK_SENSITIVITY_LOW &&
	sick_sensitivity != SICK_SENSITIVITY_HIGH ) {
      
      return false;
    }

    /* Valid */
    return true;
  }

  /**
   * \brief Indicates whether the given peak threshold is valid
   * \param sick_peak_threshold Peak threshold definition for Sick LMS 2xx
   */ 
  bool SickLMS2xx::_validSickPeakThreshold( const sick_lms_2xx_peak_threshold_t sick_peak_threshold ) const {

    /* Check the given Sick scan angle */
    if (sick_peak_threshold != SICK_PEAK_THRESHOLD_DETECTION_WITH_NO_BLACK_EXTENSION &&
	sick_peak_threshold != SICK_PEAK_THRESHOLD_DETECTION_WITH_BLACK_EXTENSION &&
	sick_peak_threshold != SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_NO_BLACK_EXTENSION &&
	sick_peak_threshold != SICK_PEAK_THRESHOLD_NO_DETECTION_WITH_BLACK_EXTENSION) {
      
      return false;
    }

    /* Valid */
    return true;
  }
  
  /**
   * \brief Indicates whether the given measuring mode is defined
   * \param sick_measuring_mode The sick measuring mode in question
   */ 
  bool SickLMS2xx::_validSickMeasuringMode( const sick_lms_2xx_measuring_mode_t sick_measuring_mode ) const {

    /* Check the given measuring mode */
    if (sick_measuring_mode != SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE &&
	sick_measuring_mode != SICK_MS_MODE_8_OR_80_REFLECTOR &&
	sick_measuring_mode != SICK_MS_MODE_8_OR_80_FA_FB_FC &&
	sick_measuring_mode != SICK_MS_MODE_16_REFLECTOR &&
	sick_measuring_mode != SICK_MS_MODE_16_FA_FB &&
	sick_measuring_mode != SICK_MS_MODE_32_REFLECTOR &&
	sick_measuring_mode != SICK_MS_MODE_32_FA &&
	sick_measuring_mode != SICK_MS_MODE_32_IMMEDIATE &&
	sick_measuring_mode != SICK_MS_MODE_REFLECTIVITY ) {

      return false;
    }
    
    /* Valid */
    return true;
  }
  
  /**
   * \brief Converts a termios baud to an equivalent Sick baud
   * \param baud_rate The baud rate to be converted to a Sick LMS baud
   * \return The Sick LMS 2xx equivalent of the given baud rate
   */
  sick_lms_2xx_baud_t SickLMS2xx::_baudToSickBaud( const int baud_rate ) const {
  
    switch(baud_rate) {
    case B9600:
      return SICK_BAUD_9600;
    case B19200:
      return SICK_BAUD_19200;
    case B38400:
      return SICK_BAUD_38400;
    case B500000:
      return SICK_BAUD_500K;
    default:
      std::cerr << "Unexpected baud rate!" << std::endl;
      return SICK_BAUD_9600;
    }
    
  }

  /**
   * \brief Converts given restart level to a corresponding string
   * \param availability_flags The availability level of the Sick LMS 2xx
   * \return The corresponding string
   */
  std::string SickLMS2xx::_sickAvailabilityToString( const uint8_t availability_flags ) const {

    /* Check if availability is specified */
    if (availability_flags == 0) {
      return "Default (Unspecified)";
    }

    std::string availability_str;
    
    /* Check if set to highest possible availability */
    if (0x01 & availability_flags) {
      availability_str += "Highest";
    }

    /* Check for real-time indices */
    if (0x02 & availability_flags) {

      /* Check whether to add a spacer */
      if (availability_str.length() > 0) {
	availability_str += ", ";
      }      

      availability_str += "Real-time indices";
    }

    /* Check for real-time indices */
    if (0x04 & availability_flags) {

      /* Check whether to add a spacer */
      if (availability_str.length() > 0) {
	availability_str += ", ";
      }

      availability_str += "No effect dazzle";
    }

    /* Return the string */
    return availability_str;
    
  }
    
  /**
   * \brief Converts restart code to a corresponding string
   * \param restart_code Restart code
   * \return The corresponding string
   */
  std::string SickLMS2xx::_sickRestartToString( const uint8_t restart_code ) const {

    std::string restart_str;

    /* Identify restart mode */
    switch(restart_code) {
    case 0x00:
      restart_str += "Restart when button actuated";
      break;
    case 0x01:
      restart_str += "Restart after set time";
      break;
    case 0x02:
      restart_str += "No restart block";
      break;
    case 0x03:
      restart_str += "Button switches field set, restart after set time";
      break;
    case 0x04:
      restart_str += "Button switches field set, no restart block";
      break;
    case 0x05:
      restart_str += "LMS2xx operates as a slave, restart after set time";
      break;
    case 0x06:
      restart_str += "LMS2xx operates as a slave, immediate restart";
      break;
    default:
      restart_str += "Unknown!";      
    }

    /* Return the restart code */
    return restart_str;
    
  }

  /**
   * \brief Converts Sick LMS temporary field code to a corresponding string
   * \param temp_field_code The temporary field code
   * \return The corresponding string
   */
  std::string SickLMS2xx::_sickTemporaryFieldToString( const uint8_t temp_field_code ) const {

    switch(temp_field_code) {
    case 0:
      return "Not used";
    case 1:
      return "Belongs to field set no. 1";
    case 2:
      return "Belongs to field set no. 2";
    default:
      return "Unknown!";
    }

  }

  /**
   * \brief Converts Sick LMS subtractive fields code to a corresponding string
   * \param subt_field_code The subtractive fields code
   * \return The corresponding string
   */
  std::string SickLMS2xx::_sickSubtractiveFieldsToString( const uint8_t subt_field_code ) const {

    switch(subt_field_code) {
    case 0:
      return "Not active";
    case 1:
      return "Active";
    default:
      return "Unknown!";
    }

  }

  /*!
   * \brief Converts Sick LMS contour function code to a corresponding string
   * \param contour_function_code The subtractive fields code
   * \return The corresponding string
   */
  std::string SickLMS2xx::_sickContourFunctionToString( const uint8_t contour_function_code ) const {

    switch(contour_function_code) {
    case 0:
      return "Not active";
    default: {

      /* For converting an int to string */
      std::ostringstream output_str;

      /* Indicate its active and include min object size */
      output_str << "Active, Min object size: " << (int)contour_function_code << " (cm)";
      return output_str.str();
      
    }
    }
    
  }
  
  /**
   * \brief Converts the Sick LMS variant to a corresponding string
   * \param sick_variant The Sick LMS variant
   * \return The corresponding string
   */
  std::string SickLMS2xx::_sickVariantToString( const unsigned int sick_variant ) const {

    /* Return the correct string */
    if(sick_variant == SICK_LMS_VARIANT_2XX_TYPE_6) {
      return "Standard device (LMS2xx,type 6)";
    }
    else if (sick_variant == SICK_LMS_VARIANT_SPECIAL) {
      return "Special device (LMS211-/221-S19/-S20)";
    }
    else {
      return "Unknown";
    }
    
  }

} //namespace SickToolbox
