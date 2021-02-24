/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gui_task_widget.hh"

#include <tf/tf.h>
#include <math.h>

#include <gazebo/msgs/msgs.hh>

#include <sstream>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUITaskWidget)

/////////////////////////////////////////////////
GUITaskWidget::GUITaskWidget()
  : GUIPlugin(),
    // setup pixmap and painter for wind compass
    //// cppcheck-suppress 3;
    windPixmap(150, 150), windPainter(&(this->windPixmap)),
    // setup pixmap and painter for contact
    contactPixmap(150, 150), contactPainter(&(this->contactPixmap)),
    // set the time for most recent contact
    contactTime(ros::Time::now())
{
#if GAZEBO_MAJOR_VERSION >= 8
  // initialize ros if that hasnt happened yet
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler |
              ros::init_options::AnonymousName);
  }

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();
  // Create the layout that sits inside the frame
  QHBoxLayout *InfoLayout = new QHBoxLayout();
  InfoLayout->setContentsMargins(0, 0, 0, 0);

  // Task info Block
  // Create a time label
  QLabel *taskInfo = new QLabel;
  // Add the label to the frame's layout
  InfoLayout->addWidget(taskInfo);
  connect(this, SIGNAL(SetTaskInfo(QString)),
      taskInfo, SLOT(setText(QString)), Qt::QueuedConnection);

  // Wind direction block
  // Create a time label
  QLabel *windDirection = new QLabel;
  // Add the label to the frame's layout
  InfoLayout->addWidget(windDirection);
  connect(this, SIGNAL(SetWindDirection(QPixmap)),
      windDirection, SLOT(setPixmap(QPixmap)), Qt::QueuedConnection);

  // Contact block
  // Create a time label
  QLabel *contact = new QLabel;
  // Add the label to the frame's layout
  InfoLayout->addWidget(contact);
  connect(this, SIGNAL(SetContact(QPixmap)),
      contact, SLOT(setPixmap(QPixmap)), Qt::QueuedConnection);

  // Add frameLayout to the frame
  mainFrame->setLayout(InfoLayout);
  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // set position and resize this widget
  this->setLayout(mainLayout);
  this->move(10, 10);
  this->resize(600, 170);

  this->node.reset(new ros::NodeHandle);
  // Subscribe to tasks topic (ROS)
  this->taskSub = this->node->subscribe("/vrx/task/info", 1,
      &GUITaskWidget::OnTaskInfo, this);
  // Subscribe to wind speed topic (ROS)
  this->windSpeedSub = this->node->subscribe("/vrx/debug/wind/speed", 1,
      &GUITaskWidget::OnWindSpeed, this);
  // Subscribe to wind direction topic (ROS)
  this->windDirectionSub = this->node->subscribe("/vrx/debug/wind/direction", 1,
      &GUITaskWidget::OnWindDirection, this);
  // Subscribe to link states topic (ROS)
  this->linkStateSub = this->node->subscribe("/gazebo/link_states", 1,
      &GUITaskWidget::OnLinkStates, this);
  // Subscribe to contact topic (ROS)
  this->contactSub = this->node->subscribe("/vrx/debug/contact", 1,
      &GUITaskWidget::OnContact, this);
#endif
}

/////////////////////////////////////////////////
GUITaskWidget::~GUITaskWidget()
{
}

/////////////////////////////////////////////////
void GUITaskWidget::OnContact(const vrx_gazebo::Contact::ConstPtr &_msg)
{
  // put red over anything preivously present
  // as to write on a red back ground
  this->contactPixmap.fill(Qt::red);
  this->contactPainter.setBrush(Qt::NoBrush);
  this->pen.setColor(Qt::black);
  this->pen.setWidth(10);
  this->contactPainter.setPen(this->pen);
  // Write Contact
  this->contactPainter.drawText(QPoint(10, 15), QString("CONTACT WITH:"));
  // Write what the wamv is in contact with
  this->contactPainter.drawText(QPoint(10, 30),
    QString::fromStdString(_msg->collision2));
  // update time of last contact
  this->contactTime = ros::Time::now();
  // send pixmap to gzserver
  this->SetContact(this->contactPixmap);
}
/////////////////////////////////////////////////
void GUITaskWidget::OnLinkStates(const gazebo_msgs::LinkStates::ConstPtr &_msg)
{
  // find wamv base_link in all the links reported
  unsigned int c = 0;
  for (auto& i : _msg->name)
  {
    if (i == "wamv::base_link")
      break;
    ++c;
  }
  // get yaw (heading) of wamv
  tf::Quaternion q(_msg->pose[c].orientation.x,
                   _msg->pose[c].orientation.y,
                   _msg->pose[c].orientation.z,
                   _msg->pose[c].orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  // update wamvHeading
  m.getRPY(roll, pitch, this->wamvHeading);
  // if the last collision was greater than 1 sec ago,
  // make contact widget all gray
  if ((ros::Time::now() - this->contactTime) > ros::Duration(1))
  {
    this->contactPixmap.fill(Qt::gray);
    this->SetContact(this->contactPixmap);
  }
}

/////////////////////////////////////////////////
void GUITaskWidget::OnWindDirection(const std_msgs::Float64::ConstPtr &_msg)
{
  // draw gray background
  this->windPixmap.fill(Qt::gray);
  // draw black circle
  this->windPainter.setBrush(Qt::NoBrush);
  this->pen.setColor(Qt::black);
  this->pen.setWidth(10);
  this->windPainter.setPen(this->pen);
  this->windPainter.drawEllipse(5, 5, 140, 140);
  // draw the N for North
  this->windPainter.setPen(Qt::red);
  this->windPainter.drawText(QRect(71, -2, 20, 20), 0, tr("N"), nullptr);

  // draw the wind hand
  const double pi = 3.14159265;
  double scale = .3*(pow(this->windSpeed, 2));
  double x = scale*cos((pi/-180)*(_msg->data)) + 75;
  double y = scale*sin((pi/-180)*(_msg->data)) + 75;
  this->pen.setWidth(10);
  this->pen.setColor(Qt::red);
  this->windPainter.setPen(this->pen);
  this->windPainter.drawLine(QLine(75, 75, x, y));
  // draw the wamv hand
  this->pen.setWidth(6);
  this->pen.setColor(Qt::blue);
  this->windPainter.setPen(this->pen);
  x = (-40*cos(this->wamvHeading + pi)) + 75;
  y = (40*sin(this->wamvHeading + pi)) + 75;
  this->windPainter.drawLine(QLine(75, 75, x, y));
  // send the compass to gzserver
  this->SetWindDirection(this->windPixmap);
}

/////////////////////////////////////////////////
void GUITaskWidget::OnWindSpeed(const std_msgs::Float64::ConstPtr &_msg)
{
  // update windSpeed
  this->windSpeed = _msg->data;
}

/////////////////////////////////////////////////
void GUITaskWidget::OnTaskInfo(const vrx_gazebo::Task::ConstPtr &_msg)
{
  std::ostringstream taskInfoStream;
  taskInfoStream.str("");
  taskInfoStream << "Task Info:\n";
  taskInfoStream << "Task Name: " << _msg->name << "\n";
  taskInfoStream << "Task Phase: " << _msg->state << "\n";
  taskInfoStream << "Ready Time: " <<
    this->FormatTime(_msg->ready_time.toSec()) << "\n";
  taskInfoStream << "Running Time: " <<
    this->FormatTime(_msg->running_time.toSec()) << "\n";
  taskInfoStream << "Elapsed Time: " <<
    this->FormatTime(_msg->elapsed_time.toSec()) << "\n";
  taskInfoStream << "Remaining Time: " <<
    this->FormatTime(_msg->remaining_time.toSec()) << "\n";
  taskInfoStream << "Timed out: ";
  if (_msg->timed_out)
    taskInfoStream << "true" << "\n";
  else
    taskInfoStream << "false" << "\n";
  taskInfoStream << "Score: " << _msg->score << "\n";
  this->SetTaskInfo(QString::fromStdString(taskInfoStream.str()));
}

/////////////////////////////////////////////////
std::string GUITaskWidget::FormatTime(unsigned int sec) const
{
  std::ostringstream stream;
  unsigned int day, hour, min;

  stream.str("");

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";

  return stream.str();
}

