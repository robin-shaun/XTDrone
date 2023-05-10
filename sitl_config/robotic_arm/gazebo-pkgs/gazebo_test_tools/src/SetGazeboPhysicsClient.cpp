#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Sets gazebo physics properties by reading settings from a ROS parameter

   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif


#include <ros/ros.h>

#include <gazebo_msgs/SetPhysicsProperties.h>

#include <string>

#define DEFAULT_MAX_STEP_SIZE 0.001
#define DEFAULT_MAX_UPDATE_RATE 1000
#define DEFAULT_PRECON_ITERS 0
#define DEFAULT_ITERS 500
#define DEFAULT_W 1.3
#define DEFAULT_RMS_ERROR_TOL 0
#define DEFAULT_CONTACT_SURFACE_LAYER 0.001
#define DEFAULT_CONTACT_MAX_CORRECTING_VEL 100
#define DEFAULT_CFM 0
#define DEFAULT_ERP 0.2
#define DEFAULT_MAX_CONTACTS 20

bool SetPhysicsProperties(ros::NodeHandle& nh,
    const std::string& gazeboPhysicsServiceTopic = "/gazebo/set_physics_properties")
{
    float time_step = DEFAULT_MAX_STEP_SIZE;
    nh.param("gazebo_physics/max_step_size", time_step, time_step);
    ROS_INFO_STREAM("Setting gazebo physics max_step_size " << time_step);

    float max_update_rate = DEFAULT_MAX_UPDATE_RATE;
    nh.param("gazebo_physics/max_update_rate", max_update_rate, max_update_rate);
    ROS_INFO_STREAM("Setting gazebo physics max_update_rate " << max_update_rate);

    float ode_slv_precon_iters = DEFAULT_PRECON_ITERS;
    nh.param("gazebo_physics/ode_slv_precon_iters", ode_slv_precon_iters, ode_slv_precon_iters);
    ROS_INFO_STREAM("Setting gazebo physics ode_slv_precon_iters " << ode_slv_precon_iters);

    float ode_slv_iters = DEFAULT_ITERS;
    nh.param("gazebo_physics/ode_slv_iters", ode_slv_iters, ode_slv_iters);
    ROS_INFO_STREAM("Setting gazebo physics ode_slv_iters " << ode_slv_iters);

    // gazebo displays this as <sor>
    float ode_slv_w = DEFAULT_W;
    nh.param("gazebo_physics/ode_slv_w", ode_slv_w, ode_slv_w);
    ROS_INFO_STREAM("Setting gazebo physics ode_slv_w " << ode_slv_w);

    float ode_slv_rms_error_tol = DEFAULT_RMS_ERROR_TOL;
//    nh.param("gazebo_physics/ode_slv_rms_error_tol",ode_slv_rms_error_tol,ode_slv_rms_error_tol);
//    ROS_INFO_STREAM("Setting gazebo physics ode_slv_rms_error_tol "<<ode_slv_rms_error_tol);

    float contact_surface_layer = DEFAULT_CONTACT_SURFACE_LAYER;
    nh.param("gazebo_physics/contact_surface_layer", contact_surface_layer, contact_surface_layer);
    ROS_INFO_STREAM("Setting gazebo physics contact_surface_layer " << contact_surface_layer);

    float contact_max_correcting_vel = DEFAULT_CONTACT_MAX_CORRECTING_VEL;
    nh.param("gazebo_physics/contact_max_correcting_vel", contact_max_correcting_vel, contact_max_correcting_vel);
    ROS_INFO_STREAM("Setting gazebo physics contact_max_correcting_vel " << contact_max_correcting_vel);

    float cfm = DEFAULT_CFM;
    nh.param("gazebo_physics/cfm", cfm, cfm);
    ROS_INFO_STREAM("Setting gazebo physics cfm " << cfm);

    float erp = DEFAULT_ERP;
    nh.param("gazebo_physics/erp", erp, erp);
    ROS_INFO_STREAM("Setting gazebo physics erp " << erp);

    float max_contacts = DEFAULT_MAX_CONTACTS;
    nh.param("gazebo_physics/max_contacts", max_contacts, max_contacts);
    ROS_INFO_STREAM("Setting gazebo physics max_contacts " << max_contacts);

    gazebo_msgs::SetPhysicsProperties srv;
    srv.request.time_step = time_step;  // dt in seconds
    srv.request.max_update_rate = max_update_rate;  //  throttle maximum physics update rate

    geometry_msgs::Vector3 gravity;  // gravity vector (e.g. earth ~[0,0,-9.81])
    gravity.x = 0;
    gravity.y = 0;
    gravity.z = -9.81;
    srv.request.gravity = gravity;

    gazebo_msgs::ODEPhysics ode_config;  // configurations for ODE

    ode_config.auto_disable_bodies = false;  // enable auto disabling of bodies, default false
    ode_config.sor_pgs_precon_iters = ode_slv_precon_iters;  // preconditioning inner iterations when uisng projected Gauss Seidel
    ode_config.sor_pgs_iters = ode_slv_iters;  // inner iterations when uisng projected Gauss Seidel
    ode_config.sor_pgs_w = ode_slv_w;  // relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
    ode_config.sor_pgs_rms_error_tol = ode_slv_rms_error_tol;  // rms error tolerance before stopping inner iterations

    ode_config.contact_surface_layer = contact_surface_layer;  // contact "dead-band" width
    ode_config.contact_max_correcting_vel = contact_max_correcting_vel;  // contact maximum correction velocity

    ode_config.cfm = cfm;  // global constraint force mixing
    ode_config.erp = erp;  // global error reduction parameter

    ode_config.max_contacts = max_contacts;  // maximum contact joints between two geoms

    srv.request.ode_config = ode_config;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>(gazeboPhysicsServiceTopic);

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call gazebo set physics service");
        return false;
    }

    if (!srv.response.success)    // return true if set wrench successful
    {
        ROS_ERROR_STREAM("Could not set gazebo physics properties. Error: '" << srv.response.status_message << "'");
        return false;
    }
    return true;
}

/**
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_set_physics_client");
    std::string gazeboPhysicsServiceTopic =  "/gazebo/set_physics_properties";
    
    if (argc > 1)
    {
        gazeboPhysicsServiceTopic = argv[1];
    }

    ros::NodeHandle nh;
    if (!SetPhysicsProperties(nh, gazeboPhysicsServiceTopic))
    {
        ROS_ERROR("Could not set physics properties.");
        return 1;
    }
    return 0;
}
