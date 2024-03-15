/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

#include <map>
#include <string>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <tf/tf.h>

#include <gazebo_plugins/gazebo_ros_bumper.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBumper)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBumper::GazeboRosBumper() : SensorPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBumper::~GazeboRosBumper()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBumper::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parentSensor = dynamic_pointer_cast<sensors::ContactSensor>(_parent);
  if (!this->parentSensor)
  {
    ROS_ERROR_NAMED("bumper", "Contact sensor parent is not of type ContactSensor");
    return;
  }

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // "publishing contact/collisions to this topic name: "
  //   << this->bumper_topic_name_ << std::endl;
  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->HasElement("bumperTopicName"))
    this->bumper_topic_name_ =
      _sdf->GetElement("bumperTopicName")->Get<std::string>();

  // "transform contact/collisions pose, forces to this body (link) name: "
  //   << this->frame_name_ << std::endl;
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("bumper", "bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("bumper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  this->contact_pub_ = this->rosnode_->advertise<gazebo_msgs::ContactsState>(
    std::string(this->bumper_topic_name_), 1);

  // Initialize
  // start custom queue for contact bumper
  this->callback_queue_thread_ = boost::thread(
      boost::bind(&GazeboRosBumper::ContactQueueThread, this));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = this->parentSensor->ConnectUpdated(
     boost::bind(&GazeboRosBumper::OnContact, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBumper::OnContact()
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosBumper::OnContact");
#endif
  if (this->contact_pub_.getNumSubscribers() <= 0)
    return;

#ifdef ENABLE_PROFILER
  IGN_PROFILE_BEGIN("fill message");
#endif
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  /// \TODO: need a time for each Contact in i-loop, they may differ
  this->contact_state_msg_.header.frame_id = this->frame_name_;
  this->contact_state_msg_.header.stamp = ros::Time(contacts.time().sec(),
                               contacts.time().nsec());

/*
  /// \TODO: get frame_name_ transforms from tf or gazebo
  /// and rotate results to local frame.  for now, results are reported in world frame.

  // if frameName specified is "world", "/map" or "map" report back
  // inertial values in the gazebo world
  physics::LinkPtr myFrame;
  if (myFrame == NULL && this->frame_name_ != "world" &&
    this->frame_name_ != "/map" && this->frame_name_ != "map")
  {
    // lock in case a model is being spawned
    boost::recursive_mutex::scoped_lock lock(
      *Simulator::Instance()->GetMRMutex());
    // look through all models in the world, search for body
    // name that matches frameName
    physics::Model_V all_models = World::Instance()->Models();
    for (physics::Model_V::iterator iter = all_models.begin();
      iter != all_models.end(); iter++)
    {
      if (*iter) myFrame =
        boost::dynamic_pointer_cast<physics::Link>((*iter)->GetLink(this->frame_name_));
      if (myFrame) break;
    }

    // not found
    if (!myFrame)
    {
      ROS_INFO_NAMED("bumper", "gazebo_ros_bumper plugin: frameName: %s does not exist"
                " yet, will not publish\n",this->frame_name_.c_str());
      return;
    }
  }
*/
  // get reference frame (body(link)) pose and subtract from it to get
  // relative force, torque, position and normal vectors
  ignition::math::Pose3d pose, frame_pose;
  ignition::math::Quaterniond rot, frame_rot;
  ignition::math::Vector3d pos, frame_pos;
  /*
  if (myFrame)
  {
    frame_pose = myFrame->WorldPose();  //-this->myBody->GetCoMPose();
    frame_pos = frame_pose.Pos();
    frame_rot = frame_pose.Rot();
  }
  else
  */
  {
    // no specific frames specified, use identity pose, keeping
    // relative frame at inertial origin
    frame_pos = ignition::math::Vector3d(0, 0, 0);
    frame_rot = ignition::math::Quaterniond(1, 0, 0, 0);  // gazebo u,x,y,z == identity
    frame_pose = ignition::math::Pose3d(frame_pos, frame_rot);
  }



  // set contact states size
  this->contact_state_msg_.states.clear();

  // GetContacts returns all contacts on the collision body
  unsigned int contactsPacketSize = contacts.contact_size();
  for (unsigned int i = 0; i < contactsPacketSize; ++i)
  {

    // For each collision contact
    // Create a ContactState
    gazebo_msgs::ContactState state;
    /// \TODO:
    gazebo::msgs::Contact contact = contacts.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i:(" << i << "/" << contactsPacketSize
      << ")     my geom:" << state.collision1_name
      << "   other geom:" << state.collision2_name
      << "         time:" << ros::Time(contact.time().sec(), contact.time().nsec())
      << std::endl;
    state.info = stream.str();

    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();

    // sum up all wrenches for each DOF
    geometry_msgs::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    unsigned int contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j)
    {
      // loop through individual contacts between collision1 and collision2
      // gzerr << j << "  Position:"
      //       << contact.position(j).x() << " "
      //       << contact.position(j).y() << " "
      //       << contact.position(j).z() << "\n";
      // gzerr << "   Normal:"
      //       << contact.normal(j).x() << " "
      //       << contact.normal(j).y() << " "
      //       << contact.normal(j).z() << "\n";
      // gzerr << "   Depth:" << contact.depth(j) << "\n";

      // Get force, torque and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)
      ignition::math::Vector3d force = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
                              contact.wrench(j).body_1_wrench().force().x(),
                            contact.wrench(j).body_1_wrench().force().y(),
                            contact.wrench(j).body_1_wrench().force().z()));
      ignition::math::Vector3d torque = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
                            contact.wrench(j).body_1_wrench().torque().x(),
                            contact.wrench(j).body_1_wrench().torque().y(),
                            contact.wrench(j).body_1_wrench().torque().z()));

      // set wrenches
      geometry_msgs::Wrench wrench;
      wrench.force.x  = force.X();
      wrench.force.y  = force.Y();
      wrench.force.z  = force.Z();
      wrench.torque.x = torque.X();
      wrench.torque.y = torque.Y();
      wrench.torque.z = torque.Z();
      state.wrenches.push_back(wrench);

      total_wrench.force.x  += wrench.force.x;
      total_wrench.force.y  += wrench.force.y;
      total_wrench.force.z  += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // transform contact positions into relative frame
      // set contact positions
      ignition::math::Vector3d position = frame_rot.RotateVectorReverse(
          ignition::math::Vector3d(contact.position(j).x(),
                                   contact.position(j).y(),
                                   contact.position(j).z()) - frame_pos);
      geometry_msgs::Vector3 contact_position;
      contact_position.x = position.X();
      contact_position.y = position.Y();
      contact_position.z = position.Z();
      state.contact_positions.push_back(contact_position);

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      ignition::math::Vector3d normal = frame_rot.RotateVectorReverse(
          ignition::math::Vector3d(contact.normal(j).x(),
                                   contact.normal(j).y(),
                                   contact.normal(j).z()));
      // set contact normals
      geometry_msgs::Vector3 contact_normal;
      contact_normal.x = normal.X();
      contact_normal.y = normal.Y();
      contact_normal.z = normal.Z();
      state.contact_normals.push_back(contact_normal);

      // set contact depth, interpenetration
      state.depths.push_back(contact.depth(j));
    }

    state.total_wrench = total_wrench;
    this->contact_state_msg_.states.push_back(state);
  }
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif
  this->contact_pub_.publish(this->contact_state_msg_);
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}


////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosBumper::ContactQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
