/*-----------------------------------------------------------
Author: Rahul Bhadani
Initial Date: January 2020

This class implements a Gazebo sensor plugin that calculates
the minimum distance of all the obstacle among all the laser
points from front laser points 

-------------------------------------------------------------*/

#include<ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>

#include <ignition/math/Box.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_plugins/gazebo_ros_laser.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/PubQueue.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <Eigen/Dense>
#include <Eigen/QR>

using namespace std;

namespace gazebo
{
    class distance: public SensorPlugin
    {
        private:

            physics::WorldPtr world; // Point to the World
            string distance_topic; // distance topic that will publish the minimum distance
            string angle_topic; // distance topic that will publish the angle of minimum distance
            string relvel_topic; // relative velocity topic
            string ns; // The Namespace

            ros::Publisher distance_publisher; // ROS publisher for minimum distance information (lead object)
            ros::Publisher relvel_publisher; // ROS publisher for relative velocity of the lead object
            ros::Publisher angle_publisher; // ROS publisher for angle of minimum distance information
            ros::NodeHandle* rosnode; // ROS Node
            sensor_msgs::LaserScan laser_data; // stores received laser scan data
            std_msgs::Float64 minimum_distance; // saves minimum distance found
            std_msgs::Float64 angle_of_min_distance; //saved the angle of minimum distance
            geometry_msgs::Twist relvel_data;
            double angle_min;
            double angle_max;

            sensors::RaySensorPtr parent_sensor;
            event::ConnectionPtr new_laserscan_connection;

            gazebo::transport::NodePtr gazebo_node;
            gazebo::transport::SubscriberPtr laser_scan_sub;

            ros::Time lastUpdate;
            std_msgs::Float64 old_dist;
            vector<double> old_relvel;
            vector<double> old_distance;
            vector<double> old_time;
            int rv_length;
            const unsigned int N_POINTS = 64;
        public:

            distance()
            {
                this->ns = "";
                this->distance_topic = this->ns + "/lead_dist";
                this->angle_topic = this->ns + "/lead_angle";
                this->relvel_topic = this->ns + "/rel_vel";
                this->angle_min = -M_PI;
                this->angle_max = M_PI;
                this->old_dist.data = 0.0;
                lastUpdate = ros::Time();
                rv_length = 0;

                old_relvel = vector<double>(N_POINTS);
                old_time = vector<double>(N_POINTS);
                old_distance = vector<double>(N_POINTS);
                for (int i = 0; i < old_relvel.size() ; ++i)
                {
                    old_relvel.at(i) = 0.0;
                    old_distance.at(i) = 0.0;
                    old_time.at(i) = 0.0;
                }

            }

            void polyfit(	const std::vector<double> &t,
                    const std::vector<double> &v,
                    std::vector<double> &coeff,
                    int order

                    )
            {
                if(rv_length+1 < N_POINTS)
                {
                    return;
                }


                // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
                Eigen::MatrixXd T(t.size(), order + 1);
                Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
                Eigen::VectorXd result;

                // check to make sure inputs are correct
                assert(t.size() == v.size());
                assert(t.size() >= order + 1);
                // Populate the matrix
                for(size_t i = 0 ; i < t.size(); ++i)
                {
                    for(size_t j = 0; j < order + 1; ++j)
                    {
                        T(i, j) = pow(t.at(i) - t.at(0), j);
                    }
                }
                //std::cout<<T<<std::endl;

                // Solve for linear least square fit
                result  = T.householderQr().solve(V);
                coeff.resize(order+1);
                for (int k = 0; k < order+1; k++)
                {
                    coeff[k] = result[k];
                }

            }	

            /* Returns nth point as average of the last n points, no argument is passed since it uses class variables*/ 
            void movingAverage(int n_iterations)
            {
                if(rv_length < N_POINTS)
                {
                    return;
                }
                double sum = 0.0;
                for (int i = 0; i < old_relvel.size() ; ++i)
                {
                    sum = sum + old_relvel.at(i);

                }
                old_relvel.at( old_relvel.size() -1 ) =  sum/rv_length;
                if (n_iterations <= 1)
                {
                    return;
                }
                else
                {
                    movingAverage(n_iterations - 1);
                }

            }

            
            double MA()
            {
                
                if(rv_length < N_POINTS-1)
                {
                 //   ROS_INFO_STREAM("rv_length: "<< rv_length);
                 //   ROS_INFO_STREAM("N_POINTS: "<< N_POINTS);
                  //  ROS_INFO_STREAM("Returning as it is ");
                    return (old_relvel.at( old_relvel.size() -1 ));
                }
                double sum = 0.0;
                for (int i = 0; i < old_relvel.size() ; ++i)
                {
                    sum = sum + old_relvel.at(i);

                }
                return(sum/rv_length);
            }

            void OnNewScan(ConstLaserScanStampedPtr &_msg)
            {
                //	ROS_INFO_STREAM("ON NEW SCAN");
                vector<double> allranges;
                this->parent_sensor->Ranges(allranges);

                int raycount = this->parent_sensor->RayCount();

                // Get the minimum distance

                double min_dist = this->parent_sensor->RangeMax();
                double range_min = this->parent_sensor->RangeMin();
                double min_angle = this->parent_sensor->AngleMin()();
                double angle_tmp = min_angle;
                double angle_incr = this->parent_sensor->AngleResolution();


                double temp_val = 100.0;
                for(vector<double>::const_iterator it = allranges.begin(); it != allranges.end(); it++, angle_tmp += angle_incr)
                {
                    if(isinf(*it))
                    {
                        temp_val = 100.0;
                    }
                    else
                    {
                        temp_val = *it;
                    }


                    if(min_dist > temp_val && temp_val > range_min && angle_tmp > this->angle_min && angle_tmp < this->angle_max)
                    {
                        min_dist = temp_val;
                        min_angle = angle_tmp;
                    }
                }
                //	ROS_INFO_STREAM("MIN DIST: "<< min_dist);

                if(isinf(min_dist))
                {
                    ROS_ERROR_STREAM("Infinity Distance Detected. Setting to zero.");
                }
                this->minimum_distance.data = min_dist;
                this->angle_of_min_distance.data = min_angle;

                ros::Time nowtime = ros::Time::now();
                ros::Duration diff = nowtime - lastUpdate;
                double elapsedTime = diff.toSec();
                double nanoSecs = diff.toNSec()*1e-9;
                //elapsedTime = elapsedTime + nanoSecs;
                lastUpdate = nowtime;

              //  ROS_INFO_STREAM("elapsedTime Sec: " << elapsedTime);
              //  ROS_INFO_STREAM("nanoSec: " << nanoSecs);


                // Check if the old time is same as new time.

                double oldtimedata = old_time.at(old_relvel.size() -1);

                /* Store the data point old_relvel array */
                if (oldtimedata != nowtime.toSec())
                {
                    for (int i = 1; i <= rv_length; ++i)
                    {
                        //			ROS_INFO_STREAM("old distance " << i <<": "<<old_distance.at(i));
                        //			ROS_INFO_STREAM("old relvel " << i <<": "<<old_relvel.at(i));
                        //			ROS_INFO_STREAM("old time " << i <<": "<<old_time.at(i));
                        old_distance.at(i-1) = old_distance.at(i);
                        old_relvel.at(i-1) = old_relvel.at(i);
                        old_time.at(i-1) = old_time.at(i);
                    }
                    old_distance.at(rv_length) = min_dist;
                    old_relvel.at(rv_length) = (min_dist - this->old_dist.data)/(elapsedTime);
                    old_time.at(rv_length) = nowtime.toSec();
                }

                if(rv_length < N_POINTS-1)
                {
                    rv_length = rv_length + 1;
                }


                #if 0
                if (rv_length > 2)
                {
                    double acceleration = (old_relvel.at(rv_length) - old_relvel.at(rv_length -1) )/(elapsedTime*0.5);
                    //		ROS_INFO_STREAM("accel = " <<acceleration);
                    // cap the relative acceleration to be within +- 3.0 m/s^2
                    if(abs(acceleration) > 3.0)
                    {
                        old_relvel.at(rv_length) = old_relvel.at(rv_length -1) + copysign(3.0*elapsedTime, acceleration);
                    }
                    if( isinf(old_relvel.at(rv_length) ))
                    {
                        ROS_ERROR_STREAM("Estimated relative velocity is set at infinity.");
                    }
                }
                #endif
                /* Store ends */

                //movingAverage(5); // 5 times movies average
                // placeholder for storing polynomial coefficient
                std::vector<double> coeff;
                std::vector<double> dist_coeff;

                #if 0 
                double vfitted = 0.0;
                double dfitted = 0.0;

                //	ROS_INFO_STREAM("rv length: "<<rv_length);
                if(rv_length >= N_POINTS-1)
                {
                    polyfit(old_time, old_relvel, coeff, 3);	
                    polyfit(old_time, old_distance, dist_coeff, 3);	

                    if(isinf(coeff[0]))
                    {
                        ROS_ERROR_STREAM("Coefficient 0 for relative veocity fitting is inf");
                    }
                    if(isinf(coeff[1]))
                    {
                        ROS_ERROR_STREAM("Coefficient 1 for relative veocity fitting is inf");
                    }
                    if(isinf(coeff[2]))
                    {
                        ROS_ERROR_STREAM("Coefficient 2 for relative veocity fitting is inf");
                    }
                    if(isinf(dist_coeff[0]))
                    {
                        ROS_ERROR_STREAM("Coefficient 1 for relative distance fitting is inf");
                    }
                    if(isinf(dist_coeff[1]))
                    {
                        ROS_ERROR_STREAM("Coefficient 2 for relative distance fitting is inf");
                    }
                    if(isinf(dist_coeff[2]))
                    {
                        ROS_ERROR_STREAM("Coefficient 0 for relative distance fitting is inf");
                    }
                    //		std::cout <<"Coefficients are "<< coeff[0] <<", "<<coeff[1] <<", "<<coeff[2] <<", "<<coeff[3]<<std::endl;   
                    for(int p = 0; p < old_time.size(); ++ p)
                    {
                        double vfitted = coeff[0] + coeff[1]*(old_time.at(p)-old_time.at(0))  + coeff[2]*(pow(old_time.at(p) -old_time.at(0) , 2)) + coeff[3]*(pow(old_time.at(p) - old_time.at(0), 3)) ;
                        double dfitted = dist_coeff[0] + dist_coeff[1]*(old_time.at(p)-old_time.at(0)) + dist_coeff[2]*(pow(old_time.at(p) -old_time.at(0) , 2)) + dist_coeff[3]*(pow(old_time.at(p) - old_time.at(0), 3)) ;
                        //		std::cout << "Old = "<< old_relvel.at(p) <<", new ="<<vfitted<<std::endl;
                        old_relvel.at(p) = vfitted;
                        old_distance.at(p) = dfitted;
                    }

                }

                if(rv_length >= N_POINTS-1)
                {
                    vfitted = coeff[0] + coeff[1]*(old_time.at( rv_length -1 )-old_time.at(0)) + coeff[2]*(pow(old_time.at( rv_length -1) -old_time.at(0) , 2)) + coeff[3]*(pow(old_time.at( rv_length -1 ) - old_time.at(0), 3)) ;
                    dfitted = dist_coeff[0] + dist_coeff[1]*(old_time.at( rv_length -1 )-old_time.at(0)) + dist_coeff[2]*(pow(old_time.at( rv_length -1) -old_time.at(0) , 2)) + dist_coeff[3]*(pow(old_time.at( rv_length -1 ) - old_time.at(0), 3)) ;
                }

                //this->relvel_data.linear.z = vfitted; // old_relvel.at( rv_length) ;
                this->relvel_data.linear.z = old_relvel.at( rv_length) ;
                if( isinf(this->relvel_data.linear.z  ))
                {
                    ROS_ERROR_STREAM("Estimated (filtered) relative velocity is set at infinity.");
                }

                //ROS_INFO_STREAM("this->relvel.linear.z: "<<this->relvel_data.linear.z);
                //this->minimum_distance.data = dfitted;

                #endif
                this->relvel_data.linear.z =  MA() ; //old_relvel.at( rv_length) ;
                this->minimum_distance.data = old_distance.at( rv_length) ;
                if( isinf(this->minimum_distance.data))
                {
                    ROS_ERROR_STREAM("Estimated (filtered) relative distance is set at infinity.");
                }

            //    ROS_INFO_STREAM("this->relvel_data.linear.z: "<<this->relvel_data.linear.z);	
            //    ROS_INFO_STREAM("this->minimum_distance.data: "<<this->minimum_distance.data);	
                this->distance_publisher.publish(this->minimum_distance);
                //ROS_INFO_STREAM("distance published");
                this->angle_publisher.publish(this->angle_of_min_distance);	
                //ROS_INFO_STREAM("angle published");
                this->relvel_publisher.publish(this->relvel_data);
                //ROS_INFO_STREAM("relvel published");
                this->old_dist.data = min_dist;
                //ROS_INFO_STREAM("last value saved:"<< min_dist);
            }

            void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
            {
                ROS_INFO_STREAM("Minimum distance estimator SensorPlugin Loaded.");
                this->gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
                this->gazebo_node->Init("default");

                // Get the name of the parent sensor
                GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
                this->parent_sensor = dynamic_pointer_cast<sensors::RaySensor>(parent);

                if(!this->parent_sensor)
                {
                    gzthrow("[distance SensorPlugin] distance plugin requires a Ray Sensor as its parent");
                }

                this->world = physics::get_world(this->parent_sensor->WorldName()); 
                string grandparent = this->parent_sensor->ParentName();
                size_t pos = grandparent.find("::");
                this->ns = "/" + grandparent.substr (0, pos); 
                ROS_INFO_STREAM("[distance SensorPlugin] Namespace retrieved is: "<<this->ns);
                this->distance_topic = this->ns + "/lead_dist";
                this->angle_topic = this->ns + "/lead_angle";
                this->relvel_topic = this->ns + "/rel_vel";

                if(sdf->HasElement("angleMin"))
                {
                    this->angle_min = sdf->GetElement("angleMin")->Get<double>();
                }
                else
                {
                    gzwarn << "[distance SensorPlugin] Using default minimum angle: " << this->angle_min << "\n";
                }
                if(sdf->HasElement("angleMax"))
                {
                    this->angle_max = sdf->GetElement("angleMax")->Get<double>();
                }
                else
                {
                    gzwarn << "[distance SensorPlugin] Using default maximum angle: " << this->angle_max << "\n";
                }

                ROS_INFO_STREAM("[distance SensorPlugin] Plugin's parent name: "<<this->parent_sensor->Name());
                ROS_INFO_STREAM("[distance SensorPlugin] Plugin's granparent: "<<this->parent_sensor->ParentName() );
                ROS_INFO_STREAM("[distance SensorPlugin] Plugin's full scope name: "<<this->parent_sensor->ScopedName() );
                this->rosnode = new ros::NodeHandle(this->ns+"/distance");
                this->distance_publisher = this->rosnode->advertise<std_msgs::Float64>(this->distance_topic, 1);
                this->angle_publisher = this->rosnode->advertise<std_msgs::Float64>(this->angle_topic, 1);
                this->relvel_publisher = this->rosnode->advertise<geometry_msgs::Twist>(this->relvel_topic, 1);

                // Subscribe Gazebo-topic and work on laser data and produce minimum of all distances.
                this->laser_scan_sub = this->gazebo_node->Subscribe(this->parent_sensor->Topic(), &distance::OnNewScan, this);
                ROS_INFO_STREAM("SENSOR PLUGIN LOADED");
            }
    };
    GZ_REGISTER_SENSOR_PLUGIN(distance)
}

