/*!
 * \file main.cc
 * \brief A simple application using the Sick LMS 1xx driver.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <stdlib.h>
#include <string>
#include <iostream>
#include <sicktoolbox/SickLMS1xx.hh>

using namespace std;
using namespace SickToolbox;

int main(int argc, char* argv[])
{
  
  /*
   * Instantiate an instance
   */
  SickLMS1xx sick_lms_1xx;

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms_1xx.Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }
  
  try {
    unsigned int status = 1;
    unsigned int num_measurements = 0;
    unsigned int range_1_vals[SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
    unsigned int range_2_vals[SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
    //sick_lms_1xx.SetSickScanFreqAndRes(SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_25,
    //SickLMS1xx::SICK_LMS_1XX_SCAN_RES_25);
    //sick_lms_1xx.SetSickScanDataFormat(SickLMS1xx::SICK_LMS_1XX_DIST_DOUBLE_PULSE,
    //				         SickLMS1xx::SICK_LMS_1XX_REFLECT_NONE);
    sick_lms_1xx.SetSickScanDataFormat(SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_16BIT);
    for (int i = 0; i < 1000; i++) {
      sick_lms_1xx.GetSickMeasurements(range_1_vals,range_2_vals,range_1_vals,range_2_vals,num_measurements,&status);
      std::cout << i << ": " << num_measurements << " " << status << std::endl;
    }
  }
  
  catch(SickConfigException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }

  catch(SickIOException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }

  catch(SickTimeoutException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }
  
  catch(...) {
    cerr << "An Error Occurred!" << endl;
    return -1;
  }
  
  
  /*
   * Uninitialize the device
   */
  try {
    sick_lms_1xx.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  /* Success! */
  return 0;

}
    
