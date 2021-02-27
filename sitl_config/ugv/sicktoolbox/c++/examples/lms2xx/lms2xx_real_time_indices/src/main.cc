/*!
 * \file main.cc
 * \brief Illustrates how to use the Sick Toolbox C++ interface
 *        to acquire scan data along with real-time indices from
 *        a Sick LMS 2xx.
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

#include <stdlib.h>
#include <string>
#include <iostream>
#include <sicktoolbox/SickLMS2xx.hh>

using namespace std;
using namespace SickToolbox;

int main(int argc, char* argv[])
{
  
  string device_str;                      
  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::SICK_BAUD_38400;

  unsigned int values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0}; // Uses macro defined in SickLMS2xx.hh
  unsigned int num_values = 0;                                   // Holds the number of measurements returned
  unsigned int telegram_idx = 0;
  unsigned int real_time_idx = 0;
  
  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms2xx_real_time_indices PATH [BAUD RATE]" << endl
	 << "Ex: lms2xx_real_time_indices /dev/ttyUSB0 9600" << endl;
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

  /*
   * Instantiate an instance
   */
  SickLMS2xx sick_lms_2xx(device_str);

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
   * Ensure real-time indices are enabled
   */
  if (sick_lms_2xx.GetSickAvailability() & SickLMS2xx::SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES) {

    try {
    
      /*
       * Acquire a few scans from the Sick LMS
       */
      for (unsigned int i=0; i < 10; i++) {
	
	/* We don't want Fields A,B, or C, so we pass NULL */
	sick_lms_2xx.GetSickScan(values,num_values,NULL,NULL,NULL,&telegram_idx,&real_time_idx);
	cout << "\t  Num. Values: " << num_values << ", Msg Idx: " << telegram_idx
	     << ", Real-time Idx: " << real_time_idx << endl;
	
      }
      
    }
    
    /* Catch anything else and exit */ 
    catch(...) {
      cerr << "An error occurred!" << endl;
      return -1;
    }

  }
    
  else {
    cout << "Please set the Sick LMS to an availability w/ real-time indices..." << endl;
    cout << "Hint: Use the lms_config utility/example! :o)"<< endl;
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
    
