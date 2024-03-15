/*
 * Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#include <boost/algorithm/string.hpp>
#include <gazebo_plugins/gazebo_ros_joint_state_publisher.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace gazebo;

GazeboRosJointStatePublisher::GazeboRosJointStatePublisher() {}

// Destructor
GazeboRosJointStatePublisher::~GazeboRosJointStatePublisher() {
    rosnode_->shutdown();
}

void GazeboRosJointStatePublisher::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    // Store the pointer to the model
    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();

    this->robot_namespace_ = parent_->GetName ();
    if ( !_sdf->HasElement ( "robotNamespace" ) ) {
        ROS_INFO_NAMED("joint_state_publisher", "GazeboRosJointStatePublisher Plugin missing <robotNamespace>, defaults to \"%s\"",
                   this->robot_namespace_.c_str() );
    } else {
        this->robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
        if ( this->robot_namespace_.empty() ) this->robot_namespace_ = parent_->GetName ();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );

    if ( !_sdf->HasElement ( "jointName" ) ) {
        ROS_ASSERT ( "GazeboRosJointStatePublisher Plugin missing jointNames" );
    } else {
        sdf::ElementPtr element = _sdf->GetElement ( "jointName" ) ;
        std::string joint_names = element->Get<std::string>();
        boost::erase_all ( joint_names, " " );
        boost::split ( joint_names_, joint_names, boost::is_any_of ( "," ) );
    }

    this->update_rate_ = 100.0;
    if ( !_sdf->HasElement ( "updateRate" ) ) {
        ROS_WARN_NAMED("joint_state_publisher", "GazeboRosJointStatePublisher Plugin (ns = %s) missing <updateRate>, defaults to %f",
                   this->robot_namespace_.c_str(), this->update_rate_ );
    } else {
        this->update_rate_ = _sdf->GetElement ( "updateRate" )->Get<double>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) {
        this->update_period_ = 1.0 / this->update_rate_;
    } else {
        this->update_period_ = 0.0;
    }
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world_->SimTime();
#else
    last_update_time_ = this->world_->GetSimTime();
#endif

    for ( unsigned int i = 0; i< joint_names_.size(); i++ ) {
        physics::JointPtr joint = this->parent_->GetJoint(joint_names_[i]);
        if (!joint) {
            ROS_FATAL_NAMED("joint_state_publisher", "Joint %s does not exist!", joint_names_[i].c_str());
        }
        joints_.push_back ( joint );
        ROS_INFO_NAMED("joint_state_publisher", "GazeboRosJointStatePublisher is going to publish joint: %s", joint_names_[i].c_str() );
    }

    ROS_INFO_NAMED("joint_state_publisher", "Starting GazeboRosJointStatePublisher Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName ().c_str() );

    tf_prefix_ = tf::getPrefixParam ( *rosnode_ );
    joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState> ( "joint_states",1000 );

#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world_->SimTime();
#else
    last_update_time_ = this->world_->GetSimTime();
#endif
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &GazeboRosJointStatePublisher::OnUpdate, this, _1 ) );
}

void GazeboRosJointStatePublisher::OnUpdate ( const common::UpdateInfo & _info )
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosCamera::OnNewFrame");
#endif
    // Apply a small linear velocity to the model.
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world_->SimTime();
#else
    common::Time current_time = this->world_->GetSimTime();
#endif
    if (current_time < last_update_time_)
    {
        ROS_WARN_NAMED("joint_state_publisher", "Negative joint state update time difference detected.");
        last_update_time_ = current_time;
    }

    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
#ifdef ENABLE_PROFILER
        IGN_PROFILE_BEGIN("publishJointStates");
#endif
        publishJointStates();
#ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
#endif
        last_update_time_+= common::Time ( update_period_ );
    }

}

void GazeboRosJointStatePublisher::publishJointStates() {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );
    joint_state_.velocity.resize ( joints_.size() );

    for ( int i = 0; i < joints_.size(); i++ ) {
        physics::JointPtr joint = joints_[i];
        double velocity = joint->GetVelocity( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position ( 0 );
#else
        double position = joint->GetAngle ( 0 ).Radian();
#endif
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = position;
        joint_state_.velocity[i] = velocity;
    }
    joint_state_publisher_.publish ( joint_state_ );
}
