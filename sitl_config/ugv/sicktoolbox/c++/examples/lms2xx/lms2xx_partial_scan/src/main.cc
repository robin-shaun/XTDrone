/*!
 * \file main.cc
 * \brief Illustrates how to acquire partial scans
 *        from the Sick LMS as well as telegram indices.
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
#include <iomanip>
#include <iostream>
#include <sicktoolbox/SickLMS2xx.hh>

using namespace std;
using namespace SickToolbox;

int main(int argc, char* argv[]) {
  
  string device_str;                      
  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::SICK_BAUD_38400;
  
  unsigned int values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0}; // Uses macro defined in SickLMS2xx.hh
  unsigned int num_values = 0;                                   // Holds the number of measurements returned
  unsigned int scan_idx = 0;                                     // Holds the idx for the returned partial scan
  unsigned int telegram_idx = 0;                                 // Holds the idx of the telegram associated w/ scan
  
  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms2xx_partial_scan PATH [BAUD RATE]" << endl
	 << "Ex: lms2xx_partial_scan /dev/ttyUSB0 9600" << endl;
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
   * Ensure real-time indices are set
   */
  if (sick_lms_2xx.GetSickAvailability() & SickLMS2xx::SICK_FLAG_AVAILABILITY_REAL_TIME_INDICES) {
  
    try {
      
      /*
       * Set the device variant to 100/0.25
       *
       * NOTE: Setting the variant this way ensures that the
       *       partial scans will start at angles that are a
       *       multiple of 0.25 deg.
       *
       */
      sick_lms_2xx.SetSickVariant(SickLMS2xx::SICK_SCAN_ANGLE_100,SickLMS2xx::SICK_SCAN_RESOLUTION_25);
      
      /*
       * Acquire some partial scans from Sick
       */
      for (unsigned int i=0; i < 12; i++) {
	
	/*
	 * NOTE: Notice that here we are also obtaining the telegram idx.  In a
	 *       proper implementation, this value would be used to ensure the
	 *       temporal consistency (acting as a sequence number) of the newly
	 *       obtained partial scan. Here we simply just print it out.
	 *
	 *       Also, we don't want Field A,B, or C outputs... so we pass in
	 *       NULL for these params.
	 */	
	sick_lms_2xx.GetSickPartialScan(values,num_values,scan_idx,NULL,NULL,NULL,&telegram_idx);
	cout << "\t  Start angle: " << setw(4) << 0.25*scan_idx << ", Num. Values: " << num_values << ", Msg Idx: " << telegram_idx << endl;
	
      }
      
    }
    
    /* Catch anything else and exit */ 
    catch(...) {
      cerr << "An error occurred!" << endl;
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
    
