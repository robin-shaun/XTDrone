/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef ROBOTICSGROUP_GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN
#define ROBOTICSGROUP_GAZEBO_PLUGINS_MIMIC_JOINT_PLUGIN

// ROS includes
#include <ros/ros.h>

// ros_control
#include <control_toolbox/pid.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
    class MimicJointPlugin : public ModelPlugin {
    public:
        MimicJointPlugin();
        ~MimicJointPlugin();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void UpdateChild();

    private:
        // Parameters
        std::string joint_name_, mimic_joint_name_, robot_namespace_;
        double multiplier_, offset_, sensitiveness_, max_effort_;
        bool has_pid_;

        // PID controller if needed
        control_toolbox::Pid pid_;

        // Pointers to the joints
        physics::JointPtr joint_, mimic_joint_;

        // Pointer to the model
        physics::ModelPtr model_;

        // Pointer to the world
        physics::WorldPtr world_;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
    };
}

#endif
