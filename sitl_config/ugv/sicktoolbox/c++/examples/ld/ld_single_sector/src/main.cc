/*!
 * \file main.cc
 * \brief A simple application illustrating the use of
 *        the Sick LD C++ driver using a single sector.
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

/* Use the namespace */
using namespace std;
using namespace SickToolbox;

int main (int argc, char *argv[]) {

  /* A string for the IP address */
  string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);

  /* Check the num of args */
  if(argc > 2 || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cerr << "Usage: ld_single_sector [SICK IP ADDRESS]" << endl
	      << "Ex. ld_single_sector 192.168.1.11" << endl;
    return -1;
  }
  
  /* Assign the IP address */
  if(argc == 2) {
    sick_ip_addr = argv[1];
  }

  /* Define the data buffers */
  double values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_values = 0;

  /* Define the bounds for a single sector */
  double sector_start_ang = 90;
  double sector_stop_ang = 270;

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

    /* Set the desired sector configuration */
    sick_ld.SetSickTempScanAreas(&sector_start_ang,&sector_stop_ang,1);
    
    /* Print the sector configuration */
    sick_ld.PrintSickSectorConfig();
    
    /* Acquire some range measurements */
    for (unsigned int i = 0; i < 10; i++) {

      /* Here we only want the range values so the second arg is NULL */
      sick_ld.GetSickMeasurements(values,NULL,&num_values);
      cout << "\t  Num. Values: " << num_values << endl;    
      
    }

  }

  /* Catch any exceptions */
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
