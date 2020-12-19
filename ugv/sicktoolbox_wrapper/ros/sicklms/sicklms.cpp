///////////////////////////////////////////////////////////////////////////////
// this program just uses sicktoolbox to get laser scans, and then publishes
// them as ROS messages
//
// Copyright (C) 2008, Morgan Quigley
//
// I am distributing this code under the BSD license:
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <sicktoolbox/SickLMS2xx.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <diagnostic_updater/diagnostic_updater.h> // Publishing over the diagnostics channels.
#include <diagnostic_updater/publisher.h>
using namespace SickToolbox;
using namespace std;

void publish_scan(diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> *pub, uint32_t *range_values,
                  uint32_t n_range_values, uint32_t *intensity_values,
                  uint32_t n_intensity_values, double scale, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  if (inverted) {
    scan_msg.angle_min = angle_max;
    scan_msg.angle_max = angle_min;
  } else {
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values-1);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (2*M_PI) * scan_msg.angle_increment;
  scan_msg.range_min = 0;
  if (scale == 0.01) {
    scan_msg.range_max = 81;
  }
  else if (scale == 0.001) {
    scan_msg.range_max = 8.1;
  }
  scan_msg.ranges.resize(n_range_values);
  scan_msg.header.stamp = start;
  for (size_t i = 0; i < n_range_values; i++) {
    // Check for overflow values, see pg 124 of the Sick LMS telegram listing
    switch (range_values[i])
    {
    // 8m or 80m operation
    case 8191: // Measurement not valid
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 8190: // Dazzling
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 8189: // Operation Overflow
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 8187: // Signal to Noise ratio too small
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 8186: // Erorr when reading channel 1
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 8183: // Measured value > maximum Value
      scan_msg.ranges[i] = numeric_limits<float>::infinity();
      break;
    ///< The following are commented out until I verify if 16m and 32m operation are used.
    /*// 16m operation
    case 16383: // Measurement not valid
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 16382: // Dazzling
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 16381: // Operation Overflow
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 16379: // Signal to Noise ratio too small
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 16378: // Erorr when reading channel 1
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 16385: // Measured value > maximum Value
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    // 32m operation
    case 32767: // Measurement not valid
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 32766 // Dazzling
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 32765: // Operation Overflow
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 32763: // Signal to Noise ratio too small
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 32762: // Erorr when reading channel 1
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;
    case 32759: // Measured value > maximum Value
      scan_msg.ranges[i] = numeric_limits<float>::quiet_NaN();
      break;*/
    default:
      scan_msg.ranges[i] = (float)range_values[i] * (float)scale;
    }
  }
  scan_msg.intensities.resize(n_intensity_values);
  for (size_t i = 0; i < n_intensity_values; i++) {
    scan_msg.intensities[i] = (float)intensity_values[i];
  }
  

  pub->publish(scan_msg);
}

SickLMS2xx::sick_lms_2xx_measuring_units_t StringToLmsMeasuringUnits(string units)
{
  if (units.compare("mm") == 0)
    return SickLMS2xx::SICK_MEASURING_UNITS_MM;
  else if (units.compare("cm") == 0)
    return SickLMS2xx::SICK_MEASURING_UNITS_CM;
  
  return SickLMS2xx::SICK_MEASURING_UNITS_UNKNOWN;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sicklms");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	string port;
	int baud;
	int delay;
	bool inverted;
	int angle;
	double resolution;
	std::string measuring_units;
	std::string frame_id;
	double scan_time = 0;
	double angle_increment = 0;
	float angle_min = 0.0;
	float angle_max = 0.0;
	
	// Diagnostic publisher parameters
	double desired_freq;
	nh_ns.param<double>("desired_frequency", desired_freq, 75.0);
	double min_freq;
	nh_ns.param<double>("min_frequency", min_freq, desired_freq);
	double max_freq;
	nh_ns.param<double>("max_frequency", max_freq, desired_freq);
	double freq_tolerance; // Tolerance before error, fractional percent of frequency.
 	nh_ns.param<double>("frequency_tolerance", freq_tolerance, 0.3);
	int window_size; // Number of samples to consider in frequency
	nh_ns.param<int>("window_size", window_size, 30);
	double min_delay; // The minimum publishing delay (in seconds) before error.  Negative values mean future dated messages.
	nh_ns.param<double>("min_acceptable_delay", min_delay, 0.0);
	double max_delay; // The maximum publishing delay (in seconds) before error.
	nh_ns.param<double>("max_acceptable_delay", max_delay, 0.2);
	std::string hardware_id;
	nh_ns.param<std::string>("hardware_id", hardware_id, "SICK LMS");
	double time_offset_sec;
 	nh_ns.param<double>("time_offset", time_offset_sec, 0.0);
	ros::Duration time_offset(time_offset_sec);
	
	// Set up diagnostics
	diagnostic_updater::Updater updater;
	updater.setHardwareID(hardware_id);
	diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> scan_pub(nh.advertise<sensor_msgs::LaserScan>("scan", 10), updater, 
	    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, freq_tolerance, window_size),
	    diagnostic_updater::TimeStampStatusParam(min_delay, max_delay));
	
	nh_ns.param("port", port, string("/dev/lms200"));
	nh_ns.param("baud", baud, 38400);
	nh_ns.param("connect_delay", delay, 0);
	nh_ns.param("inverted", inverted, false);
	nh_ns.param("angle", angle, 0);
	nh_ns.param("resolution", resolution, 0.0);
	nh_ns.param("units", measuring_units, string());
	nh_ns.param<std::string>("frame_id", frame_id, "laser");
	
  // Check if use_rep_117 is set.
  std::string key;
  if (nh.searchParam("use_rep_117", key))
  {
    ROS_ERROR("The use_rep_117 parameter has not been specified and is now ignored.  This parameter was removed in Hydromedusa.  Please see: http://ros.org/wiki/rep_117/migration");
  }

	SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::IntToSickBaud(baud);
	if (desired_baud == SickLMS2xx::SICK_BAUD_UNKNOWN)
	{
		ROS_ERROR("Baud rate must be in {9600, 19200, 38400, 500000}");
		return 1;
	}
	uint32_t range_values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
  uint32_t intensity_values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
	uint32_t n_range_values = 0;
  uint32_t n_intensity_values = 0;
	SickLMS2xx sick_lms(port);
	double scale = 0;
  double angle_offset;
  uint32_t partial_scan_index;

	try
	{
		uint32_t on_delay = 0;
		if(delay > 0){
		  on_delay = delay;
		}
		sick_lms.Initialize(desired_baud, on_delay);

                // Set the angle and resolution if possible (not an LMSFast) and
                // the user specifies a setting.
                int actual_angle = sick_lms.GetSickScanAngle();
                double actual_resolution = sick_lms.GetSickScanResolution();
                SickLMS2xx::sick_lms_2xx_measuring_units_t actual_units = sick_lms.GetSickMeasuringUnits();

		// Attempt to set measurement angles and angular resolution
                try {
                  if ((angle && actual_angle != angle) || (resolution && actual_resolution != resolution)) {
                     ROS_INFO("Setting variant to (%i, %f)", angle, resolution);
                     sick_lms.SetSickVariant(sick_lms.IntToSickScanAngle(angle),
                                             sick_lms.DoubleToSickScanResolution(resolution));
                  } else {
                    ROS_INFO("Variant setup not requested or identical to actual (%i, %f)", actual_angle,
                      actual_resolution);
                    angle = actual_angle;
                    resolution = actual_resolution;
                  }
                } catch (SickConfigException e) {
                  actual_angle = sick_lms.GetSickScanAngle();
                  actual_resolution = sick_lms.GetSickScanResolution();
                  if (angle != actual_angle) {
                    ROS_WARN("Unable to set scan angle. Using %i instead of %i.", actual_angle, angle);
                    angle = actual_angle;
                  }
                  if (resolution != actual_resolution) {
                    ROS_WARN("Unable to set resolution. Using %e instead of %e.", actual_resolution, resolution);
                    resolution = actual_resolution;
                  }
                }
                
                // Attempt to set measurement output mode to cm or mm
                try {
		  if (!measuring_units.empty() && (actual_units != StringToLmsMeasuringUnits(measuring_units))) {
                    ROS_INFO("Setting measuring units to '%s'", measuring_units.c_str());
                    actual_units = StringToLmsMeasuringUnits(measuring_units);
                    sick_lms.SetSickMeasuringUnits(actual_units);
                  } else {
                    ROS_INFO("Measuring units setup not requested or identical to actual ('%s')",
                      sick_lms.SickMeasuringUnitsToString(actual_units).c_str());
                  }
		}  catch (SickConfigException e) {
		  actual_units = sick_lms.GetSickMeasuringUnits();
		  if (StringToLmsMeasuringUnits(measuring_units) != actual_units) {
                    ROS_WARN("Unable to set measuring units. Using '%s' instead of '%s'.",
                      sick_lms.SickMeasuringUnitsToString(actual_units).c_str(), measuring_units.c_str());
                    measuring_units = sick_lms.SickMeasuringUnitsToString(actual_units);
		  }
		}

		if (actual_units == SickLMS2xx::SICK_MEASURING_UNITS_CM)
			scale = 0.01;
		else if (actual_units == SickLMS2xx::SICK_MEASURING_UNITS_MM)
			scale = 0.001;
		else
		{
			ROS_ERROR("Invalid measuring unit.");
			return 1;
		}

    // The scan time is always 1/75 because that's how long it takes
    // for the mirror to rotate. If we have a higher resolution, the
    // SICKs interleave the readings, so the net result is we just
    // shift the measurements.
    if (angle == 180 || sick_lms.IsSickLMS2xxFast()) {
      scan_time = 1.0 / 75;
    } 
    else {
      SickLMS2xx::sick_lms_2xx_scan_resolution_t scan_resolution =
        SickLMS2xx::DoubleToSickScanResolution(resolution);
      if ( scan_resolution == SickLMS2xx::SICK_SCAN_RESOLUTION_25) {
        // 0.25 degrees
        scan_time = 4.0 / 75;   // 53.33 ms
      }
      else if ( scan_resolution == SickLMS2xx::SICK_SCAN_RESOLUTION_50) {
        // 0.5 degrees
        scan_time = 2.0 / 75;   // 26.66 ms
      }
      else if ( scan_resolution == SickLMS2xx::SICK_SCAN_RESOLUTION_100) {
        // 1 degree
        scan_time = 1.0 / 75;   // 13.33 ms
      }
      else {
        ROS_ERROR("Bogus scan resolution.");
        return 1;
      }
      if ( scan_resolution != SickLMS2xx::SICK_SCAN_RESOLUTION_100) {
        ROS_WARN("You are using an angle smaller than 180 degrees and a "
                 "scan resolution less than 1 degree per scan. Thus, "
                 "you are in inteleaved mode and the returns will not "
                 "arrive sequentially how you read them. So, the "
                 "time_increment field will be misleading. If you need to "
                 "know when the measurement was made at a time resolution "
                 "better than the scan_time, use the whole 180 degree "
                 "field of view.");
      }
    }

    // The increment for the slower LMS is still 1.0 even if its set to
    // 0.5 or 0.25 degrees resolution because it is interleaved. So for
    // 0.5 degrees, two scans are needed, offset by 0.5 degrees. These
    // show up as two seperate LaserScan messages.
    angle_increment = sick_lms.IsSickLMS2xxFast() ? 0.5 : 1.0;

    angle_offset = (180.0-angle)/2;
	}
	catch (...)
	{
		ROS_ERROR("Initialize failed! are you using the correct device path?");
    return 2;
	}
	try
	{
		while (ros::ok())
		{
      if (sick_lms.IsSickLMS2xxFast()) {
        // There's no inteleaving, but we can capture both the range
        // and intensity simultaneously
        sick_lms.GetSickScan(range_values, intensity_values,
                             n_range_values, n_intensity_values);
        angle_min = -M_PI/4;
        angle_max = M_PI/4;
      }
      else if (angle != 180) {
        // If the angle is not 180, we can't get partial scans as they
        // arrive. So, we have to wait for a full scan to get
        // there. 
        sick_lms.GetSickScan(range_values, n_range_values);
        angle_min = (-90.0 + angle_offset) * M_PI / 180.0;
        angle_max = (90.0 - angle_offset)  * M_PI / 180.0;
      }
      else {
        // We get scans that could be potentially interleaved
        // depending on the mode. We want to stream out the data as
        // soon as we get it otherwise the timing won't work right to
        // reconstruct the data if the sensor is moving.
        sick_lms.GetSickPartialScan(range_values, n_range_values,
                                    partial_scan_index);
        double partialScanOffset = 0.25 * partial_scan_index;
        angle_min = (-90.0 + angle_offset + partialScanOffset) * M_PI / 180.0;
        angle_max = (90.0 - angle_offset - fmod(1.0 - partialScanOffset, 1.0))
          * M_PI / 180.0;
      }
      // Figure out the time that the scan started. Since we just
      // fished receiving the data, we'll assume that the mirror is at
      // 180 degrees now, or half a scan time.
      // Add user provided time offset to handle constant latency.
      ros::Time end_of_scan = ros::Time::now();
      ros::Time start = end_of_scan - ros::Duration(scan_time / 2.0) + time_offset;

			publish_scan(&scan_pub, range_values, n_range_values, intensity_values,
                   n_intensity_values, scale, start, scan_time, inverted,
                   angle_min, angle_max, frame_id);
			ros::spinOnce();
			// Update diagnostics
			updater.update();
		}
	}
	catch (...)
	{
		ROS_ERROR("Unknown error.");
		return 1;
	}
	try
	{
		sick_lms.Uninitialize();
	}
	catch (...)
	{
		ROS_ERROR("Error during uninitialize");
		return 1;
	}
	ROS_INFO("Success.\n");

	return 0;
}

