/*!
 * \file main.cc
 * \brief Illustrates how to work with the Sick LD
 *        using multiple scan areas/sectors.
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

/* Define the number of active sectors */
#define NUM_ACTIVE_SECTORS (3)

using namespace std;
using namespace SickToolbox;

int main (int argc, char *argv[]) {

  /* A string for the IP address */
  string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);
  
  /* Check the num of args */
  if(argc > 2 || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cerr << "Usage: ld_multi_sector [SICK IP ADDRESS]" << endl
	      << "Ex. ld_multi_sector 192.168.1.11" << endl;
    return -1;
  }
  
  /* Assign the IP address */
  if(argc == 2) {
    sick_ip_addr = argv[1];
  }
  
  /* Define the temporal scan area (3 active sectors/scan areas)
   * NOTE: This scan configuration will persist until the power
   *       is cycled or until it is reset w/ different params.
   */
  double sector_start_angs[NUM_ACTIVE_SECTORS] = {45,270,345};
  double sector_stop_angs[NUM_ACTIVE_SECTORS] = {90,315,15};
  
  /* Define the destination buffers */
  double range_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int reflect_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};

  /* Some buffer for additional info we can get from the Sick LD */
  unsigned int num_values[NUM_ACTIVE_SECTORS] = {0};
  unsigned int data_offsets[NUM_ACTIVE_SECTORS] = {0};
  unsigned int sector_ids[NUM_ACTIVE_SECTORS] = {0};

  /* Define the object */
  SickLD sick_ld(sick_ip_addr);

  /* Initialize the Sick LD */
  try {
    sick_ld.Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }
  
  try {
  
    /* Set the temporary scan areas */
    sick_ld.SetSickTempScanAreas(sector_start_angs,sector_stop_angs,NUM_ACTIVE_SECTORS);
    
    /* Print the sector configuration */
    sick_ld.PrintSickSectorConfig();
    
    /* Request some measurements */
    for (unsigned int i = 0; i < 10; i++) {

      /* Acquire the most recent range and reflectivity (echo amplitude) values */
      sick_ld.GetSickMeasurements(range_values,reflect_values,num_values,sector_ids,data_offsets);
      
      /* Print out some data for each sector */
      for (unsigned int i = 0; i < NUM_ACTIVE_SECTORS; i++) {
	
	cout << "\t[Sector ID: " << sector_ids[i]
	     << ", Num Meas: " << num_values[i]
	     << ", 1st Range Val: " << range_values[data_offsets[i]]
	     << ", 1st Reflect Val: " << reflect_values[data_offsets[i]]
	     << "]" << endl;
	
      }
      cout << endl;
	
    }

  }

  /* Catch any exception! */
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
  
  /* Success !*/
  return 0;

}
