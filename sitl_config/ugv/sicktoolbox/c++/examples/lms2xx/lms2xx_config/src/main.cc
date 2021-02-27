/*!
 * \file main.cc
 * \brief A simple config utility for Sick LMS 2xx LIDARs.
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

/* Implementation dependencies */
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <sicktoolbox/SickLMS2xx.hh>

#define INVALID_OPTION_STRING "   Invalid option!!! :o("
#define PROMPT_STRING "lms2xx?> "

using namespace std;
using namespace SickToolbox;

/**
 * \fn sigintHandler
 * \brief Callback for SIGINT interrupt. Used for uninitialization.
 * \param signal Signal ID
 */
void sigintHandler(int signal);

/**
 * \fn strToInt
 * \brief Utility function for converting string to int
 * \param input_str Input string to convert
 */
int strToInt(string input_str);

/**
 * \fn getUserOption
 * \brief Utility function for grabbing user input
 * \param is_null_input Output parameter indicating whether
 *                      user entered NULL input
 */
int getUserOption(bool &is_null_input);

/**
 * \fn writeToEEPROM
 * \brief Confirms the user actually wants to perform write
 */
bool writeToEEPROM();

/**
 * \fn setMeasuringUnits
 * \brief Prompts the user and sets the desired measuring
 *        units via the driver interface.
 */
void setMeasuringUnits();

/**
 * \fn setMeasuringMode
 * \brief Prompts the user and sets the desired measuring
 *        mode via the driver interface.
 */
void setMeasuringMode();

/**
 * \fn setAvailabilityLevel
 * \brief Prompts the user and sets the desired availability
 *        level via the driver interface.
 */
void setAvailabilityLevel();

/**
 * \fn setSensitivityLevel
 * \brief Prompts the user and sets the desired sensitivity
 *        level via the driver interface.
 */
void setSensitivityLevel();

/* A pointer to the Sick obj */
SickLMS2xx *sick_lms_2xx = NULL;

int main(int argc, char * argv[]) {

  string device_str; // Device path of the Sick LMS 2xx
  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::SICK_BAUD_38400;

  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms2xx_config PATH [BAUD RATE]" << endl
	 << "Ex: lms2xx_config /dev/ttyUSB0 9600" << endl;
    return -1;
  }

  /* Only device path is given */
  if (argc == 2) {
    device_str = argv[1];
  }

  /* Device path and baud are given */
  if (argc == 3) {    
    device_str = argv[1];
    if ((desired_baud = SickLMS2xx::StringToSickBaud(argv[2])) == SickLMS2xx::SICK_BAUD_UNKNOWN) {
      cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
      return -1;
    }
  }
  
  /* Instantiate the SickLMS2xx class with the device path string. */
  sick_lms_2xx = new SickLMS2xx(device_str);

  cout << endl;
  cout << "The Sick LIDAR C++/Matlab Toolbox    " << endl;
  cout << "Sick LMS 2xx Config Utility          " << endl;
  
  /* Initialize the device */
  try {
    sick_lms_2xx->Initialize(desired_baud);
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
    return -1;
  }
  
  /* Register the signal handler */
  signal(SIGINT,sigintHandler);

  cout << "NOTE - Configuring the Sick LMS - REQUIRES - writing to the device's EEPROM." << endl;
  cout << "       The number of times this can be done is finite (a few thousand)." << endl;
  cout << "       Thus, you should configure sparingly." << endl;
  cout << endl;

  do {

    cout << "Enter your choice: (Ctrl-c to exit)" << endl;
    cout << "  [1] Set measuring units"<< endl;
    cout << "  [2] Set measuring mode"<< endl;
    cout << "  [3] Set availability level" << endl;
    cout << "  [4] Set sensitivity level" << endl;
    cout << "  [5] Show detailed configuration" << endl;
    cout << PROMPT_STRING;
    
    bool is_null_input;
    switch(getUserOption(is_null_input)) {

    case 1:
      setMeasuringUnits();
      break;
    case 2:
      setMeasuringMode();
      break;
    case 3:
      setAvailabilityLevel();
      break;
    case 4:
      setSensitivityLevel();
      break;
    case 5:
      sick_lms_2xx->PrintSickConfig();
      break;
    default:	
      if (!is_null_input) {
	cerr << INVALID_OPTION_STRING << endl;
      }
    }
    
    cout << endl;
    
  } while(true);
	 
  /* Success! */
  return 0;

}

/**
 * Handle the SIGINT signal
 */
void sigintHandler( int signal ) {

  cout << endl;
  cout << "Quitting..." << endl;

  /* Unitialize the device */
  try {

    sick_lms_2xx->Uninitialize();
    delete sick_lms_2xx;

  }

  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    exit(-1);
  }

  cout << endl;
  cout << "Thanks for using the Sick LIDAR Matlab/C++ Toolbox!" << endl;
  cout << "Bye Bye :o)" << endl;
  
  exit(0);

}

/**
 * Converts ascii string to an integer
 */
int strToInt( string input_str ) {
  int int_val;
  istringstream input_stream(input_str);
  input_stream >> int_val;
  return int_val;
}

/**
 * Reads in user input string and returns numeric
 * representation
 */
int getUserOption( bool &is_null_input ) {

  string user_input_str;
  getline(cin,user_input_str);

  // Check whether its null input
  is_null_input = true;
  if (user_input_str.length() > 0) {
    is_null_input = false;
  }

  return strToInt(user_input_str);
  
}

/**
 * Prompts the user to confirm requested operation
 */
bool writeToEEPROM( ) {

  string user_input_str;
  
  do {

    cout << "This will attempt to write to EEPROM. Continue [y/n]? ";
    getline(cin,user_input_str);

    // Check whether its null input
    if (user_input_str == "Y" || user_input_str == "y") {
      return true;
    }
    else if (user_input_str == "N" || user_input_str == "n") {
      return false;
    }
    else {
      cerr << "Please answer [y/n]!" << endl;
    }
    
  } while (true);
    
}

/**
 * Sets the measuring units of the device
 */
void setMeasuringUnits() {

  bool keep_running = true;
  
  do {

    cout << endl;
    cout << "Select the desired units:" << endl;
    cout << "  [1] Centimeters (cm)"<< endl;
    cout << "  [2] Millimeters (mm)"<< endl;
    cout << "  [3] Back to main"<< endl;
    cout << PROMPT_STRING;

    try {

      bool is_null_input;
      switch(getUserOption(is_null_input)) {
	
      case 1:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS units to (cm)" << endl;
	  sick_lms_2xx->SetSickMeasuringUnits(SickLMS2xx::SICK_MEASURING_UNITS_CM);
	  cout << "  Done!" << endl;
	}
	break;
      case 2:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS units to (mm)" << endl;
	  sick_lms_2xx->SetSickMeasuringUnits(SickLMS2xx::SICK_MEASURING_UNITS_MM);
	  cout << "  Done!" << endl;
	}
	break;
      case 3:
	keep_running = !keep_running;
	break;
      default:
	if (!is_null_input) {
	  cerr << INVALID_OPTION_STRING << endl;
	}	
      }
      
    }
    
    catch( SickException &sick_exception ) {
      cerr << "A Sick exception occurred!" << endl;
      exit(-1);
    }
    
    catch(...) {
      cerr << "An unknown exception occurred!" << endl;
      exit(-1);
    } 
    
  } while (keep_running);

}

/**
 * Sets the measuring mode of the device
 */
void setMeasuringMode() {

  bool keep_running = true;
  
  do {

    cout << endl;
    cout << "Select the desired measuring mode (see telegram listing for descriptions):" << endl;
    cout << "  [1] Measurement range 8m/80m; field A, field B, and dazzle" << endl;
    cout << "  [2] Measurement range 8m/80m; reflector bits in 8 levels" << endl;	
    cout << "  [3] Measurement range 8m/80m; field A, field B and field C" << endl;
    cout << "  [4] Measurement range 16m/theoretically 160m; reflector bits in 4 levels" << endl;
    cout << "  [5] Measurement range 16m/theoretically 160m; field A and field B" << endl;
    cout << "  [6] Measurement range 32m/theoretically 320m; reflector bits in 2 levels" << endl;
    cout << "  [7] Measurement range 32m/theoretically 320m; field A" << endl;
    cout << "  [8] Measurement range 32m/theoretically 320m; Immediate" << endl;
    cout << "  [9] Reflectivity/Intensity values" << endl;
    cout << " [10] Back to main" << endl;
    cout << PROMPT_STRING;
    
    try {

      bool is_null_input;
      switch(getUserOption(is_null_input)) {
	
      case 1:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 8m/80m; field A, field B, and dazzle" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_8_OR_80_FA_FB_DAZZLE);
	  cout << "  Done!" << endl;
	}
	break;
      case 2:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 8m/80m; reflector bits in 8 levels" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_8_OR_80_REFLECTOR);
	  cout << "  Done!" << endl;
	}
	break;
      case 3:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 8m/80m; field A, field B and field C" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_8_OR_80_FA_FB_FC);
	  cout << "  Done!" << endl;
	}
	break;
      case 4:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 16m/theoretically 160m; reflector bits in 4 levels" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_16_REFLECTOR);
	  cout << "  Done!" << endl;
	}
	break;
      case 5:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 16m/theoretically 160m; field A and field B" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_16_FA_FB);
	  cout << "  Done!" << endl;
	}
	break;
      case 6:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 32m/theoretically 320m; reflector bit in 2 levels" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_32_REFLECTOR);
	  cout << "  Done!" << endl;
	}
	break;
      case 7:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 32m/theoretically 320m; field A" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_32_FA);
	  cout << "  Done!" << endl;
	}
	break;
      case 8:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Measurement range 32m/theoretically 320m; Immediate" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_32_IMMEDIATE);
	  cout << "  Done!" << endl;
	}
	break;
      case 9:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Meas. Mode to: Reflectivity/Intensity" << endl;
	  sick_lms_2xx->SetSickMeasuringMode(SickLMS2xx::SICK_MS_MODE_REFLECTIVITY);
	  cout << "  Done!" << endl;
	}
	break;
      case 10:
	keep_running = !keep_running;
	break;
      default:
	if (!is_null_input) {
	  cerr << INVALID_OPTION_STRING << endl;
	}
      }
      
    }
    
    catch( SickException &sick_exception ) {
      cerr << "A Sick exception occurred!" << endl;
      exit(-1);
    }
    
    catch(...) {
      cerr << "An unknown exception occurred!" << endl;
      exit(-1);
    } 
   
  } while (keep_running);

}

/**
 * Sets the measuring mode of the device
 */
void setAvailabilityLevel() {

  bool keep_running = true;
  
  do {

    cout << endl;
    cout << "Select the desired availability (see telegram listing for descriptions):" << endl;
    cout << "  [1] Restore to factory default" << endl;
    cout << "  [2] High" << endl;	
    cout << "  [3] High w/ Real-time indices" << endl;
    cout << "  [4] High w/ No effect dazzle" << endl;
    cout << "  [5] High w/ Real-time indices and No effect dazzle" << endl;
    cout << "  [6] Real-time indices" << endl;
    cout << "  [7] Real-time indices w/ No effect dazzle" << endl;
    cout << "  [8] No effect dazzle" << endl;
    cout << "  [9] Back to main" << endl;
    cout << PROMPT_STRING;
    
    try {

      bool is_null_input;
      switch(getUserOption(is_null_input)) {
	
      case 1:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: Factory settings" << endl;
	  sick_lms_2xx->SetSickAvailability();
	  cout << "  Done!" << endl;
	}
	break;
      case 2:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: High" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_HIGH);
	  cout << "  Done!" << endl;
	}
	break;
      case 3:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: High w/ Real-time indices" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_HIGH | SickLMS2xx::SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES);
	  cout << "  Done!" << endl;
	}
	break;
      case 4:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: High w/ No effect dazzle" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_HIGH | SickLMS2xx::SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT);
	  cout << "  Done!" << endl;
	}
	break;
      case 5:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: High w/ Real-time indices and No effect dazzle" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_HIGH | SickLMS2xx::SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES | SickLMS2xx::SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT);
	  cout << "  Done!" << endl;
	}
	break;
      case 6:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: Real-time indices" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES);
	  cout << "  Done!" << endl;
	}
	break;
      case 7:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: Real-time indices w/ No effect dazzle" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES | SickLMS2xx::SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT);
	  cout << "  Done!" << endl;
	}
	break;
      case 8:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Availability to: No effect dazzle" << endl;
	  sick_lms_2xx->SetSickAvailability(SickLMS2xx::SICK_FLAG_AVAILABILITY_DAZZLE_NO_EFFECT);
	  cout << "  Done!" << endl;
	}
	break;
      case 9:
	keep_running = !keep_running;
	break;
      default:
	if (!is_null_input) {
	  cerr << INVALID_OPTION_STRING << endl;
	}
      }
      
    }
    
    catch( SickException &sick_exception ) {
      cerr << "A Sick exception occurred!" << endl;
      exit(-1);
    }
    
    catch(...) {
      cerr << "An unknown exception occurred!" << endl;
      exit(-1);
    } 
   
  } while (keep_running);
  
}

/**
 * Sets the Sick LMS sensitivity level
 */
void setSensitivityLevel() {

  bool keep_running = true;
  
  do {

    cout << endl;
    cout << "Select the desired sensitivity level:" << endl;
    cout << "  [1] High (42m @ 10% reflectivity)"<< endl;
    cout << "  [2] Standard (30m @ 10% reflectivity, factory setting)"<< endl;
    cout << "  [3] Medium (25m @ 10% reflectivity)"<< endl;
    cout << "  [4] Low (20m @ 10% reflectivity)"<< endl;
    cout << "  [5] Back to main"<< endl;
    cout << PROMPT_STRING;

    try {

      bool is_null_input;
      switch(getUserOption(is_null_input)) {
	
      case 1:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Sensitivity to: High" << endl;
	  sick_lms_2xx->SetSickSensitivity(SickLMS2xx::SICK_SENSITIVITY_HIGH);
	  cout << "  Done!" << endl;
	}
	break;
      case 2:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Sensitivity to: Standard (Factory setting)" << endl;
	  sick_lms_2xx->SetSickSensitivity();
	  cout << "  Done!" << endl;
	}
	break;
      case 3:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Sensitivity to: Medium" << endl;
	  sick_lms_2xx->SetSickSensitivity(SickLMS2xx::SICK_SENSITIVITY_MEDIUM);
	  cout << "  Done!" << endl;
	}
	break;
      case 4:
	if (writeToEEPROM()) {
	  cout << "  Setting Sick LMS Sensitivity to: Low" << endl;
	  sick_lms_2xx->SetSickSensitivity(SickLMS2xx::SICK_SENSITIVITY_LOW);
	  cout << "  Done!" << endl;
	}
	break;
      case 5:
	keep_running = !keep_running;
	break;
      default:
	if (!is_null_input) {
	  cerr << INVALID_OPTION_STRING << endl;
	}	
      }
      
    }
    
    catch( SickException &sick_exception ) {
      cerr << "A Sick exception occurred!" << endl;
      exit(-1);
    }
    
    catch(...) {
      cerr << "An unknown exception occurred!" << endl;
      exit(-1);
    } 
    
  } while (keep_running);
  
}
