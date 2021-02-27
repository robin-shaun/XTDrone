/*!
 * \file main.cc
 * \brief Illustrates how to set the device variant and then
 *        acquire measured values
 *
 * Note: This example WILL NOT WORK for LMS 211-S14, 221-S14,
 *       291-S14 models as they do not support variant switching.
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
#include <iostream>
#include <sicktoolbox/SickLMS2xx.hh>

/* Use the namespace */
using namespace std;
using namespace SickToolbox;

int main(int argc, char * argv[]) {
  
  string device_str;                
  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::SICK_BAUD_38400;
  
  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms2xx_set_variant PATH [BAUD RATE]" << endl
	 << "Ex: lms2xx_set_variant /dev/ttyUSB0 9600" << endl;
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
  SickLMS2xx sick_lms_2xx(device_str);

  /* Define some buffers to hold the returned measurements */
  unsigned int values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_values = 0;

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms_2xx.Initialize(desired_baud);
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
    return -1;
  }  

  /*
   * Check whether device is LMS Fast
   */
  if (!sick_lms_2xx.IsSickLMS2xxFast()) {

    try {
      
      /*
       * Set the device variant to 100/0.25
       *
       * NOTE: If an invalid variant definition is
       *       given a SickConfigException will be
       *       thrown stating so.
       *
       */
      cout << "\tSetting variant to 100/0.25" << std::endl << flush;
      sick_lms_2xx.SetSickVariant(SickLMS2xx::SICK_SCAN_ANGLE_100,SickLMS2xx::SICK_SCAN_RESOLUTION_25);
      
      /*
       * Acquire some measurements from Sick LMS 2xx using 100/0.25
       */
      cout << "\tAcquiring some measurements..." << endl;
      for(unsigned int i = 0; i < 10; i++) {
	
	/* Acquire the most recent scan from the Sick */
	sick_lms_2xx.GetSickScan(values,num_values);
	
	/* Display the number of measurements */
	cout << "\t  Num. Values: " << num_values << endl;
	
      }
      
      /*
       * Set the device variant to 180/0.5
       */
      cout << std::endl << "\tSetting variant to 180/0.50" << endl;
      sick_lms_2xx.SetSickVariant(SickLMS2xx::SICK_SCAN_ANGLE_180,SickLMS2xx::SICK_SCAN_RESOLUTION_50);
      
      /*
       * Acquire some measurements from Sick LMS 2xx using 180/0.50
       */
      cout << "\tAcquiring some measurements..." << endl;
      for(unsigned int i = 0; i < 10; i++) {
	
	/* Acquire the most recent scan from the Sick */
	sick_lms_2xx.GetSickScan(values,num_values);
	
	/* Display the number of measured values */
	cout << "\t  Num. Values: " << num_values << endl;
	
      }
      
    }

    catch(...) {
      cerr << "An error occurred!" << endl;
    }

  }
    
  else {
    cerr << "Oops... Your Sick is an LMS Fast!" << endl;
    cerr << "It doesn't support the variant command." << endl;
  }
    
  /*
   * Uninitialize the device
   */
  try {
    sick_lms_2xx.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  /* Success! */
  return 0;

}
