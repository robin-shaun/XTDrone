// Written By : Yossi Cohen
#define MY_GAZEBO_VER 5
// If the plugin is not defined then define it
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <random>
// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_config.h>
// Ignition
#include <ignition/math.hh>
// ROS Communication
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
// Boost Thread Mutex
#include <boost/thread/mutex.hpp>
// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <colorado/coloradoConfig.h>
#include <boost/bind.hpp> // Boost Bind

#include <tf/transform_broadcaster.h>

// Maximum time delay before a "no command" behaviour is initiated.
#define command_MAX_DELAY 0.3

#define PI 3.14159265359
#define VehicleLength 3.5932
#define VehicleWidth 1.966
#define WheelRadius 0.497
#define HP 190 //190 HP @3400 rpm=142KW @3400 rpm & 515 NM @1300
#define POWER 142
#define TRANSMISSIONS 4
#define IDLE_RPM 550
//#define MY_GAZEBO_VER 5

namespace gazebo
{

class coloradoDrivingPlugin : public ModelPlugin
{
  double deltaSimTime = 0.001;
  ///  Constructor
public:
  coloradoDrivingPlugin() {}

  /// The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
  {
    std::cout << "My major GAZEBO VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;
    this->model = _model;
    // Store the pointers to the joints
    this->left_wheel_1 = this->model->GetJoint("left_wheel_1");
    this->left_wheel_2 = this->model->GetJoint("left_wheel_2");
    this->right_wheel_1 = this->model->GetJoint("right_wheel_1");
    this->right_wheel_2 = this->model->GetJoint("right_wheel_2");
    this->streer_joint_left_1 = this->model->GetJoint("steering_joint_left_1");
    this->streer_joint_right_1 = this->model->GetJoint("steering_joint_right_1");
    this->spring_left_1 = this->model->GetJoint("spring_left_1");
    this->spring_right_1 = this->model->GetJoint("spring_right_1");
    this->spring_left_2 = this->model->GetJoint("spring_left_2");
    this->spring_right_2 = this->model->GetJoint("spring_right_2");

    // Starting Timers
    Throttle_command_timer.Start();
    Angular_command_timer.Start();
    Breaking_command_timer.Start();
    this->Ros_nh = new ros::NodeHandle("coloradoDrivingPlugin_node");

    // Subscribe to the topic, and register a callback
    Steering_rate_sub = this->Ros_nh->subscribe("colorado/Driving/Steering", 60, &coloradoDrivingPlugin::On_Steering_command, this);
    Velocity_rate_sub = this->Ros_nh->subscribe("colorado/Driving/Throttle", 60, &coloradoDrivingPlugin::On_Throttle_command, this);
    Breaking_sub = this->Ros_nh->subscribe("colorado/Driving/Break", 60, &coloradoDrivingPlugin::On_Break_command, this);

    platform_hb_pub_ = this->Ros_nh->advertise<std_msgs::Bool>("colorado/ConnctionStatus", 60);
    platform_Speedometer_pub = this->Ros_nh->advertise<std_msgs::Float64>("colorado/Speedometer", 60);

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&coloradoDrivingPlugin::OnUpdate, this, _1));
    std::cout << "Setting up dynamic config" << std::endl;

    this->model_reconfiguration_server = new dynamic_reconfigure::Server<colorado::coloradoConfig>(*(this->Ros_nh));
    this->model_reconfiguration_server->setCallback(boost::bind(&coloradoDrivingPlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    std::cout << "Dynamic configuration is set up" << std::endl;
    Steering_Request = 0;
    Throttle_command = 0;
    std::cout << "Driving Plugin Loaded" << std::endl;
  }

public:
  void dynamic_Reconfiguration_callback(colorado::coloradoConfig &config, uint32_t level)
  {
    control_P = config.Steer_control_P;
    control_I = config.Steer_control_I;
    control_D = config.Steer_control_D;
    steeringSpeed = config.Steering;
    damping = config.Damping;
    friction = config.Friction;
    power = config.Power;
    suspenSpring = config.Spring;
    SuspenDamp = config.Damper;
    SuspenTarget = config.Target;
  }

  // Called by the world update start event, This function is the event that will be called every update
public:
  void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
  {

    deltaSimTime = simInfo.simTime.Double() - simTime.Double();
    simTime = simInfo.simTime;
    // std::cout << "update function started"<<std::endl;
    // std::cout << "command_timer = " << command_timer.GetElapsed().Float() << std::endl;
    // Applying effort to the wheels , brakes if no message income
    if (Throttle_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Applies 0 throtle
      Throttle_command = 0;
    }
    if (Breaking_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Brakes
      BreakPedal = 1;
    }
    if (Angular_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Restores straight direction.
      Steering_Request = 0;
    }
    // std::cout << "Applying efforts"<<std::endl;

    EngineCalculations();
    apply_efforts();
    apply_steering();
    ApplySuspension();
    BreakPedal = 0;
    breaker();

    // std::cout << this->spring_right_1->GetAngle(0).Radian() << std::endl;
    SpeedMsg.data = Speed * 3.6;
    platform_Speedometer_pub.publish(SpeedMsg);
    connection.data = true;
    platform_hb_pub_.publish(connection);
    tf::Transform transform;
	  transform.setOrigin( tf::Vector3(model->WorldPose().Pos().X(), model->WorldPose().Pos().Y(), model->WorldPose().Pos().Z()));
	  transform.setRotation(tf::Quaternion(model->WorldPose().Rot().X(),model->WorldPose().Rot().Y(),model->WorldPose().Rot().Z(),model->WorldPose().Rot().W()));

	  TF_Broadcast(transform, "WORLD", model->GetName(), simInfo.simTime);

  }
  	void TF_Broadcast(tf::Transform transform, std::string frame_id, std::string child_frame_id, common::Time time)
	{
		 static tf::TransformBroadcaster br;
		 tf::StampedTransform st(transform, ros::Time::now()/*(time.sec, time.nsec)*/, frame_id, child_frame_id);
		 br.sendTransform(st);

	}
  void EngineCalculations()
  {
    ThrottlePedal = ThrottlePedal + deltaSimTime * 5 * (Throttle_command - ThrottlePedal); //move the gas pedal smoothly
    if (Throttle_command < 0 && Speed > 5)
      ThrottlePedal = 0;
    CurrentRPM = fabs(Speed) * GearRatio[CurrentGear] * 3.1 / (WheelRadius * 2 * PI / 60) + IDLE_RPM; //Calculating RPM from wheel speed. 3.1 being the Final Drive Gear ratio.
    if (CurrentGear < TRANSMISSIONS && Speed * 3.6 > ShiftSpeed[CurrentGear]) //simulating Torque drop in transmission
    {
      CurrentGear++;
      ShiftTime = simTime.Double() + 0.25;
    }
    else if (CurrentGear > 1 && Speed * 3.6 < ShiftSpeed[CurrentGear - 1] - 10) //simulating Torque drop in transmission
    {
      CurrentGear--;
      ShiftTime = simTime.Double() + 0.25;
    }
    int indexRPM = ((int)CurrentRPM) / 600;
    double interpolatedEngineTorque = 400 + 0.1620123 * CurrentRPM - 0.00005657748 * CurrentRPM * CurrentRPM; //An extracted function from the TorqueRPM600 array
    // double interpolatedEngineTorque=(TorqueRPM600[indexRPM]+TorqueRPM600[indexRPM+1]*fmod(CurrentRPM,600)/600)/(1+fmod(CurrentRPM,600)/600); Deprecated interpolation code
    Torque = ThrottlePedal * interpolatedEngineTorque * GearRatio[CurrentGear] * power;
    if (simTime < ShiftTime)  //simulating Torque drop in transmission
      Torque *= 0.5;
    EngineLoad = Torque;
    // std::cout << CurrentRPM << " RPM at Gear " << CurrentGear << " Speed " << Speed * 3.6 << " Engine Torque " << EngineLoad << std::endl;
  }
  void apply_efforts()
  {
    double WheelTorque = Torque;
    // std::cout << " Controlling wheels"<< std::endl;
    wheel_controller(this->left_wheel_1, WheelTorque);
    wheel_controller(this->left_wheel_2, WheelTorque);
    wheel_controller(this->right_wheel_1, WheelTorque);
    wheel_controller(this->right_wheel_2, WheelTorque);
    // std::cout << " Controlling Steering"<< std::endl;
  }
  void wheel_controller(physics::JointPtr wheelJoint, double Torque2)
  {
    WheelPower = Torque2;

    double wheelOmega = wheelJoint->GetVelocity(0);
    if (abs(wheelOmega > 1))
      jointforce = WheelPower - damping * wheelOmega - friction * wheelOmega / fabs(wheelOmega);
    else
      jointforce = WheelPower - damping * wheelOmega;
    wheelJoint->SetForce(0, jointforce);
    if (wheelJoint == right_wheel_2)
    {
      wheelsSpeedSum = wheelsSpeedSum + wheelOmega;
      Speed = wheelsSpeedSum * WheelRadius / 4;
      wheelsSpeedSum = 0;
    }
    else
      wheelsSpeedSum = wheelsSpeedSum + wheelOmega;
  }
  void apply_steering()
  {
    double ThetaAckerman = 0;
    double ThetaOuter = 0;
    if (Steering_Request > 0) //turning right
    {
      ThetaAckerman = atan(1 / ((1 / (tan(Steering_Request)) + (VehicleWidth / VehicleLength))));
      steer_controller(this->streer_joint_left_1, Steering_Request);
      steer_controller(this->streer_joint_right_1, ThetaAckerman);
    }
    else if (Steering_Request < 0) //turning left
    {
      ThetaAckerman = atan(1 / ((1 / (tan(-Steering_Request)) + (VehicleWidth / VehicleLength))));
      steer_controller(this->streer_joint_left_1, -ThetaAckerman);
      steer_controller(this->streer_joint_right_1, Steering_Request);
    }
    else
    {
      steer_controller(this->streer_joint_left_1, 0);
      steer_controller(this->streer_joint_right_1, 0);
    }

    //  std::cout << ThetaAckerman << std::endl;
  }
  void steer_controller(physics::JointPtr steer_joint, double Angle)
  {
    // std::cout << " getting angle"<< std::endl;
    double currentWheelAngle = steer_joint->Position(0);
    double steeringOmega = steer_joint->GetVelocity(0);
    if (steer_joint == this->streer_joint_left_1)
    {
      DesiredAngle = DesiredAngle + steeringSpeed * deltaSimTime * (Angle - DesiredAngle);
      if (fabs(Angle - DesiredAngle)<0.01)DesiredAngle=Angle;
      IerL+=DesiredAngleR - currentWheelAngle;
      double jointforce = control_P * (DesiredAngle - currentWheelAngle)+control_I*IerL - control_D * (steeringOmega);
      steer_joint->SetForce(0, jointforce);
      //  std::cout << currentWheelAngle<< std::endl;
    }
    else
    {
      DesiredAngleR = DesiredAngleR + steeringSpeed * deltaSimTime * (Angle - DesiredAngleR);
      if (fabs(Angle - DesiredAngleR)<0.01)DesiredAngleR=Angle;
      IerR+=DesiredAngleR - currentWheelAngle;
      double jointforce = control_P * (DesiredAngleR - currentWheelAngle)+control_I*IerR - control_D * (steeringOmega);
      steer_joint->SetForce(0, jointforce);
    }
    // std::cout << "efforting"<< std::endl;
    // this->jointController->SetJointPosition(steer_joint, Angle*0.61);
  }
  void breaker()
  {

    // std::cout << " getting angle"<< std::endl;
    if (BreakPedal >= 0.09 && !Breaks)
    {
      TempDamping = damping;
      damping = 10000 * BreakPedal * BreakPedal;
      Breaks = true;
      std::cout << "Break on " << damping << std::endl;
    }
    else if (BreakPedal == 0 && Breaks)
    {
      damping = TempDamping;
      Breaks = false;
      std::cout << "Breaks off " << damping << std::endl;
    }
    if (BreakPedal >= 0.09 && Breaks)
      damping = 10000 * BreakPedal * BreakPedal;

    // std::cout << "efforting"<< std::endl;
    // this->jointController->SetJointPosition(steer_joint, Angle*0.61);
  }
  void ApplySuspension()
  {
    Suspension(spring_left_1);
    Suspension(spring_left_2);
    Suspension(spring_right_1);
    Suspension(spring_right_2);
  }
  void Suspension(physics::JointPtr Suspension)
  { //The function to control the suspension of the Vehicle
    double SpringForce = -(Suspension->Position(0) + SuspenTarget) * suspenSpring - Suspension->GetVelocity(0) * SuspenDamp;
    Suspension->SetForce(0, SpringForce);
  }

  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Throttle_command(const std_msgs::Float64ConstPtr &msg)
  {

    Throttle_command_mutex.lock();
    // Recieving referance velocity
    if (msg->data > 1)
    {
      Throttle_command = 1;
    }
    else if (msg->data < -1)
    {
      Throttle_command = -1;
    }
    else
    {
      Throttle_command = msg->data;
    }

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
    Throttle_command_timer.Reset();
#endif
    Throttle_command_timer.Start();

    Throttle_command_mutex.unlock();
  }
  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Steering_command(const std_msgs::Float64ConstPtr &msg)
  {
    Angular_command_mutex.lock();
    // Recieving referance steering angle
    if (msg->data > 1)
    {
      Steering_Request = 1;
    }
    else if (msg->data < -1)
    {
      Steering_Request = -1;
    }
    else
    {
      Steering_Request = msg->data;
    }
    Steering_Request = -Steering_Request;

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
    Angular_command_timer.Reset();
#endif
    Angular_command_timer.Start();
    Angular_command_mutex.unlock();
  }
  void On_Break_command(const std_msgs::Float64ConstPtr &msg)
  {
    Breaking_command_mutex.lock();
    // Recieving referance velocity
    if (msg->data >= 1)
      BreakPedal = 1;
    else if (msg->data >= 0.09)
      BreakPedal = msg->data;
    else
      BreakPedal = 0;

// Reseting timer every message
#if GAZEBO_MAJOR_VERSION >= 5
    Breaking_command_timer.Reset();
#endif
    Breaking_command_timer.Start();
    Breaking_command_mutex.unlock();
  }

  // Defining private Pointer to model
  physics::ModelPtr model;

  // Defining private Pointer to joints

  physics::JointPtr right_wheel_1;
  physics::JointPtr right_wheel_2;
  physics::JointPtr left_wheel_1;
  physics::JointPtr left_wheel_2;
  physics::JointPtr streer_joint_left_1;
  physics::JointPtr streer_joint_right_1;
  physics::JointPtr spring_left_1;
  physics::JointPtr spring_right_1;
  physics::JointPtr spring_left_2;
  physics::JointPtr spring_right_2;

  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;

  // Defining private Ros Subscribers
  ros::Subscriber Steering_rate_sub;
  ros::Subscriber Velocity_rate_sub;
  ros::Subscriber Breaking_sub;
  // Defining private Ros Publishers
  ros::Publisher platform_hb_pub_;
  ros::Publisher platform_Speedometer_pub;
  std_msgs::Bool connection;
  std_msgs::Float64 SpeedMsg;

  // Defining private Timers
  common::Timer Throttle_command_timer;
  common::Timer Angular_command_timer;
  common::Timer Breaking_command_timer;
  common::Time simTime;

  // Defining private Mutex
  boost::mutex Angular_command_mutex;
  boost::mutex Throttle_command_mutex;
  boost::mutex Breaking_command_mutex;
  //helper vars
  float Throttle_command = 0;
  float ThrottlePedal = 0;
  float Steering_Request = 0;
  float BreakPedal = 0;
  double friction = 0;
  double WheelPower = 0;
  double DesiredAngle = 0;
  double DesiredAngleR = 0;
  double wheelsSpeedSum = 0;
  double CurrentRPM = 0;
  double ShiftTime = 0;
  double EngineLoad = 0;
  int CurrentGear = 1;
  double Torque = 0;
  double jointforce = 0;
  float tempTime = 0;
  float Speed = 0;
  bool Breaks = false;
  double IerL=0;
  double IerR=0;
  double ShiftSpeed[TRANSMISSIONS] = {0, 20, 40, 65};
  double GearRatio[TRANSMISSIONS + 1] = {0, 3.2, 2.5, 1.5, 0.8};
  double TorqueRPM600[8] = {0, 350, 515, 500, 450, 300, 200, 200};
  double PowerRPM600[8] = {0, 10, 60, 80, 120, 142, 120, 120};
  //Dynamic Configuration Definitions
  dynamic_reconfigure::Server<colorado::coloradoConfig> *model_reconfiguration_server;
  double control_P, control_I, control_D, steeringSpeed,
      damping, power, TempDamping, suspenSpring, SuspenDamp, SuspenTarget;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(coloradoDrivingPlugin)
}
// 4*(0.8654*50+0.65450*50+0.48924*50+0.48924*50)+0.48924*80*4-0.48924*50*2+2000 is 0.863347 which is the CoM height.
// LeftRearWheelLoc = (1.79660 1.04 0.48924)