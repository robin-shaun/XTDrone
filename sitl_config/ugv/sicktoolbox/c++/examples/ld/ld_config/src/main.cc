/*!
 * \file main.cc
 * \brief A simple configuration utility for Sick LD LIDARs.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * ConfigFile was written by Richard J. Wagner
 * Download at http://www-personal.umich.edu/~wagnerr/ChE.html
 *
 * LDConfig is a simple utility to make configuring the SICK LD-LRS
 * easier. By editing the configuration file, one can set up the LD-LRS
 * in different configurations, without mucking about in source code.
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <sicktoolbox/SickLD.hh>
#include <stdlib.h>
#include "ConfigFile.h"

#define INVALID_OPTION_STRING        "   Invalid option!!! :o(" 
#define PROMPT_STRING                                   "ld?> " 

/* Config file parameters */
#define CONFIG_OPT_MOTOR_SPD_STR          "SICK_LD_MOTOR_SPEED"
#define CONFIG_OPT_SCAN_AREA_STR           "SICK_LD_SCAN_AREAS"
#define CONFIG_OPT_SCAN_RES_STR       "SICK_LD_SCAN_RESOLUTION"

using namespace std;
using namespace SickToolbox;

/**
 * \fn sigintHandler
 * \brief Callback for SIGINT interrupt. Used for uninitialization.
 * \param signal Signal ID
 */
void sigintHandler(int signal);

/**
 * \fn getUserOption
 * \brief A utility function for capturing input from the user
 * \param is_null_input Indicates whether the user entered null input
 * \return User option as an integer
 */
int getUserOption(bool &is_null_input);

/**
 * \fn getFilename
 * \brief Prompts the user for config filename
 * \return Configuration file path
 */
string getFilename();

/**
 * \fn setConfig
 * \brief Attempts to set the desired config via the driver.
 *        Parses configuration file as well.
 */
void setConfig();

/**
 * \fn printConfig
 * \brief Prints the current Sick status/config
 */
void printConfig();

/**
 * \fn parseScanAreasStr
 * \brief Parses the scan areas input string.
 * \param start_angs Array of angles to begin a sector
 * \param stop_angs Array of angles to end a sector
 * \param areas String of sectors (e.g. "[0 10.75] [40.5 60.0] [85 100]")
 * \param num_pairs Number of angle pairs found
 * \return true on success, false otherwise
 *
 * pre:  all parameters != NULL
 * post: start_angs and stop_angs hold num_pairs values;
 *       0 < num_pairs < SickLD::SICK_MAX_NUM_MEASURING_SECTORS
 *
 * This function separates the scan areas found in the config file
 * into two arrays - one with angles that the scan areas begin at,
 * and another with the angles that the scan areas end at. Uses 
 * setAngle()
 */
bool parseScanAreasStr(string& scan_areas_str, double *start_angs, double *stop_angs, int& num_sectors);

/**
 * \fn parseNumStr
 * \param entry String to be converted to a number
 * \param num Variable to receive converted string
 * \return true on success, false otherwise
 *
 * Parses entry string into a double, and saves it in num.
 */
bool parseNumStr(const string& entry, double& num);

/* A pointer to the current driver instance */
SickLD *sick_ld = NULL;

int main(int argc, char* argv[])
{
  
  string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);  // IP address of the Sick LD unit

  /* Check the num args */
  if(argc > 2 || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cerr << "Usage: ld_config [SICK IP ADDRESS]" << endl
	      << "Ex. ld_config 192.168.1.11" << endl;
    return -1;
  }
  
  /* Assign the IP address */
  if(argc == 2) {
    sick_ip_addr = argv[1];
  }

  /* Instantiate the SickLD driver */
  sick_ld = new SickLD(sick_ip_addr);

  cout << endl;
  cout << "The Sick LIDAR C++/Matlab Toolbox    " << endl;
  cout << "Sick LD Config Utility          " << endl;
  cout << endl;

  /* Initialize the Sick LD */
  try {
    sick_ld->Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }

  /* Register the signal handler */
  signal(SIGINT,sigintHandler);

  do {

    cout << "Enter your choice: (Ctrl-c to exit)" << endl;
    cout << "  [1] Set new configuration"<< endl;
    cout << "  [2] Show current settings"<< endl;
    cout << PROMPT_STRING;
    
    bool is_null_input;
    switch(getUserOption(is_null_input)) {

    case 1:
      setConfig();
      break;
    case 2:
      printConfig();
      break;
    default:	
      if(!is_null_input) {
	cerr << INVALID_OPTION_STRING << endl;
      }

    }
    
    cout << endl;
    
  } while(true);
  
  /* Success */
  return 0;
  
}

/**
 * Handles the SIGINT signal
 */
void sigintHandler(int signal) {

  cout << endl;
  cout << "Quitting..." << endl;

  /* Unitialize the device */
  try {

    sick_ld->Uninitialize();
    delete sick_ld;

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
 * Reads user input string and returns numeric representation.
 */
int getUserOption(bool &is_null_input) {

  string user_input_str;
  getline(cin,user_input_str);

  // Check whether its null input
  is_null_input = true;
  if (user_input_str.length() > 0) {
    is_null_input = false;
  }

  int int_val = 0;
  istringstream input_stream(user_input_str);
  input_stream >> int_val;

  return int_val;
  
}

/**
 * Prompts the user for the filename
 */
string getFilename() {

  string filename;

  cout << "config file: ";
  getline(cin,filename);

  return filename;
  
}

/**
 * Parses config file and loads new configuration
 */
void setConfig() {

  int motor_spd;
  int num_sectors;
  double scan_res;
  double start_angs[SickLD::SICK_MAX_NUM_MEASURING_SECTORS] = {0};
  double stop_angs[SickLD::SICK_MAX_NUM_MEASURING_SECTORS] = {0};
  string scan_areas_str;
  
  ConfigFile sick_config_file;	 // Extracts values from config file
  string config_path;

  /* Prompt the user for the filename */
  config_path = getFilename();

  /* Instantiate the parser */
  if(!ifstream(config_path.c_str())) {
    sick_config_file = ConfigFile(config_path);
  }
  else {
    cerr << "Invalid filename!" << endl;
    return;
  }

  /* Use the ConfigFile class to extract the various parameters for
   * the sick configuration.
   *
   * NOTE: The third parameter specifies the value to use, if the second 
   *       parameter is not found in the file.
   */
  if(!sick_config_file.readInto(motor_spd,CONFIG_OPT_MOTOR_SPD_STR)) {
    cerr << "ERROR: Invalid config file - " << CONFIG_OPT_MOTOR_SPD_STR << " needs to be specified!" << endl;
    return;
  }
  
  if(!sick_config_file.readInto(scan_res,CONFIG_OPT_SCAN_RES_STR)) {
    cerr << "ERROR: Invalid config file - " << CONFIG_OPT_SCAN_RES_STR << " needs to be specified!" << endl;
    return;
  }

  if(!sick_config_file.readInto(scan_areas_str,CONFIG_OPT_SCAN_AREA_STR)) {
    cerr << "ERROR: Invalid config file - " << CONFIG_OPT_SCAN_AREA_STR << " needs to be specified!" << endl;
    return;
  }

  /* Extract the start/stop pairs and angle res for the scanning sectors */
  if (!parseScanAreasStr(scan_areas_str,start_angs,stop_angs,num_sectors)) {
    cerr << "ERROR: Parser failed to extract scan areas!" << endl;
    return;
  }

  /* Set the global parameters - we pass them all at once to ensure a feasible config */
  cout << endl << "\tAttempting to configure the Sick LD..." << endl;
      
  try {
    sick_ld->SetSickGlobalParamsAndScanAreas(motor_spd,scan_res,start_angs,stop_angs,num_sectors);
  }

  catch(SickConfigException &sick_config_exception) {
    cerr << "ERROR: Couldn't set requested configuration!" << endl;
    return;
  }
  
  catch(...) {
    exit(-1);
  }
  
  cout << "\t\tConfiguration Successfull!!!" << endl;
  
}

/**
 * Print the current Sick LD configuration
 */
void printConfig() {

  cout << endl;
  sick_ld->PrintSickStatus();
  sick_ld->PrintSickIdentity();
  sick_ld->PrintSickGlobalConfig();
  sick_ld->PrintSickEthernetConfig();
  sick_ld->PrintSickSectorConfig();
  
}

/**
 * Parses scan areas string in order extract desired sector configuration
 */
bool parseScanAreasStr(string& areas, double * start_angs, double * stop_angs, int& num_pairs) {
  
  unsigned long i;		// number of sectors found so far
  unsigned int start_pos = 0;	// starting position of a sector in 'areas'
  unsigned int end_pos = 0;	// ending position of a sector in 'areas'
  unsigned int split = 0;	// position of the delimiter inside a sector
  string pair;	                // a string of 'areas' that contains one start and one stop angle

  /* Get the beginning and end of the first scan sector */
  start_pos = areas.find('[',start_pos); // Find the next [, starting at start_pos
  end_pos = areas.find(']',start_pos);

  /* Keep getting sectors until we either run out of open
   * brackets or we exceed the number of allowed sectors
   */
  for(i=0; (start_pos != (unsigned int)string::npos) && (i <= SickLD::SICK_MAX_NUM_MEASURING_SECTORS); i++) {

    pair = areas.substr(start_pos+1,end_pos-(start_pos+1));

    /* Eliminate any padding before first value */
    try {      
      pair = pair.substr(pair.find_first_not_of(' '));    
    }

    catch(...) {
      cerr << "ERROR: There was an problem parsing your scan areas! Check your config file." << endl;
      return false;
    }

    split = pair.find(' ');

    /* Catch a lack of ' ' inside the sector definition;
     * alternative is to let it fail at pair.substr(split)
     */
    if(split == (unsigned int)string::npos) {
      cerr << "ERROR: Invalid sector definition." << endl;
      return false;
    }

    /* Get the number from the beginning to the delimiter */
    if(!parseNumStr(pair.substr(0,split),start_angs[i])) {
      cerr << "ERROR: Invalid start angle found." << endl;
      return false;
    }

    /* Get the number from the delimiter to the end */
    if(!parseNumStr(pair.substr(split),stop_angs[i])) {
      cerr << "ERROR: Invalid stop angle found." << endl;
      return false;
    }

    start_pos = end_pos; // Shift to the end of the current sector

    /* Try to find another sector */
    start_pos = areas.find("[", start_pos);
    end_pos = areas.find("]", start_pos);

  }

  num_pairs = i;

  /* Check if we broke out because of too many loops */
  if(num_pairs > SickLD::SICK_MAX_NUM_MEASURING_SECTORS) {
    cerr << "ERROR: Too many scan areas found (max " << SickLD::SICK_MAX_NUM_MEASURING_SECTORS << ")" << endl;
    return false;
  }

  /* Or, we might not have even entered the loop */
  if(num_pairs == 0) {
    cerr << "ERROR: No scan areas found! Check brackets in your config file." << endl;
    return false;
  }

  /* Success! */
  return true;
  
}

/**
 * Parses entry string into a double, and saves it in num.
 */
bool parseNumStr(const string& entry, double& num)
{

  string num_str = entry.substr(entry.find_first_not_of(' '));  
  istringstream input_stream(num_str.c_str());  
  if(!(input_stream >> num)) {
    cerr << "ERROR: Invalid angle value: " + num_str << endl;
    return false;
  }

  /* Success! */
  return true;

}
