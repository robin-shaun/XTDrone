
/*!
 * \file main.cc
 * \brief Illustrates a variety of ways to configure the flash
 *        parameters on the Sick LD device as well as how to set
 *        the unit's clock.
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
#include <sicktoolbox/SickLD.hh>

using namespace std;
using namespace SickToolbox;

int main (int argc, char *argv[]) {

  /* A string for the IP address */
  string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);

  /* Check the num of args */
  if(argc > 2 || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cerr << "Usage: ld_more_config [SICK IP ADDRESS]" << endl
	      << "Ex. ld_more_config 192.168.1.11" << endl;
    return -1;
  }
  
  /* Assign the IP address */
  if(argc == 2) {
    sick_ip_addr = argv[1];
  }
  
  /* Define the object */
  SickLD sick_ld(sick_ip_addr);

  /*
   * Initialize the Sick LD
   */
  try {
    sick_ld.Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }
  
  try {
  
    /* Assign absolute and then relative time */
    //uint16_t new_sick_time = 0;
    //sick_ld.SetSickTimeAbsolute(1500,new_sick_time);
    //cout << "\tNew sick time: " << new_sick_time << endl;    
    //sick_ld.SetSickTimeRelative(-500,new_sick_time);
    //cout << "\tNew sick time: " << new_sick_time << endl;
    
    /* Configure the Sick LD sensor ID */
    //sick_ld.PrintSickGlobalConfig();
    //sick_ld.SetSickSensorID(16);
    //sick_ld.PrintSickGlobalConfig();
    
    /* Configure the sick motor speed */
    //sick_ld.PrintSickGlobalConfig();
    //sick_ld.SetSickMotorSpeed(10);
    //sick_ld.PrintSickGlobalConfig();
    
    /* Configure the sick scan resolution */
    //sick_ld.PrintSickGlobalConfig();
    //sick_ld.SetSickScanResolution(0.5);
    //sick_ld.PrintSickGlobalConfig();
    
    /* Configure all the global parameters */
    //double start_angle = 45;
    //double stop_angle = 315;
    //sick_ld.PrintSickGlobalConfig();
    //sick_ld.PrintSickSectorConfig();
    //sick_ld.SetSickGlobalParamsAndScanAreas(10,0.5,&start_angle,&stop_angle,1);
    //sick_ld.PrintSickGlobalConfig();
    //sick_ld.PrintSickSectorConfig();
    
  }

  catch(...) {
    cerr << "An error occurred!" << endl;
  }

  /*
   * Uninitialize the device
   */
  try {
    sick_ld.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
    
  /* Success! */
  return 0;

}
