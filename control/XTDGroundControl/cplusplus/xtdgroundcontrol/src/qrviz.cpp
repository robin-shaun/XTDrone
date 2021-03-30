#include "../include/xtdgroundcontrol/qrviz.hpp"
#include <QDebug>

qrviz::qrviz(QVBoxLayout* layout)
{
    // create rviz
    render_panel = new rviz::RenderPanel();
    layout->addWidget(render_panel);
    manager = new rviz::VisualizationManager(render_panel);
    tool_manager = manager->getToolManager();
    ROS_ASSERT(manager!=NULL);
    render_panel->initialize(manager->getSceneManager(),manager);  // must before mananer->init
    //init rviz
    manager->initialize();
    manager->startUpdate();
    manager->removeAllDisplays();

}
void qrviz::Set_FixedFrame(QString Fixed_name)
{
    manager->setFixedFrame(Fixed_name);
    qDebug()<<manager->getFixedFrame();
}
void qrviz::Display_Grid(int Cell_count, QColor color, bool enable)
{
    if (Grid!=NULL)  // only one grid is added on rviz
    {
        delete Grid;
        Grid = NULL;
    }
   Grid = manager->createDisplay("rviz/Grid","myGrid",enable);
   Grid->subProp("Plane Cell Count")->setValue(Cell_count);
   Grid->subProp("Color")->setValue(color);
   ROS_ASSERT(Grid!=NULL);
   manager->startUpdate();
}
void qrviz::Display_Pose(QString pose_topic, QString pose_shape, QColor color, bool enable, int num)
{
    if (count_pose_num[num]==0)
    {
        count_pose_num[num]++;
        Pose.push_back(NULL);
    }
    else
    {
        if (Pose[num]!=NULL)  // only one grid is added on rviz
        {
            delete Pose[num];
            Pose[num] = NULL;
        }
    }
   Pose[num] = manager->createDisplay("rviz/Pose","myPose",enable);
   Pose[num]->subProp("Topic")->setValue(pose_topic);
   Pose[num]->subProp("Color")->setValue(color);
   Pose[num]->subProp("Shape")->setValue(pose_shape);
   ROS_ASSERT(Pose[num]!=NULL);
   manager->startUpdate();
}
void qrviz::Display_TF(bool enable1, bool enable)
{
    if (Tf!=NULL)  // only one grid is added on rviz
    {
        delete Tf;
        Tf = NULL;
    }
   Tf = manager->createDisplay("rviz/TF","myTF",enable);
   Tf->subProp("Show Names")->setValue(enable1);
   ROS_ASSERT(Tf!=NULL);
   manager->startUpdate();
}
void qrviz::Display_LaserScan(QString laser_topic, bool enable)
{
    if (Laser!=NULL)  // only one grid is added on rviz
    {
        delete Laser;
        Laser = NULL;
    }
   Laser = manager->createDisplay("rviz/LaserScan","myLaser",enable);
   Laser->subProp("Topic")->setValue(laser_topic);
   ROS_ASSERT(Laser!=NULL);
   manager->startUpdate();
}
void qrviz::Display_Image(QString Image_topic, bool enable, int num)
{
    qDebug()<<"image_num"<<num;
    if (count_image_num[num]==0)
    {
        count_image_num[num]++;
        Image.push_back(NULL);
    }
    else
    {
        if (Image[num]!=NULL)  // only one grid is added on rviz
        {
            delete Image[num];
            Image[num] = NULL;
        }
    }
   Image[num] = manager->createDisplay("rviz/Image","myImage",enable);
   Image[num]->subProp("Image Topic")->setValue(Image_topic);
   ROS_ASSERT(Image[num]!=NULL);
   manager->startUpdate();
}
//void qrviz::Set_start_pose()
//{
//    rviz::Tool* current_tool = tool_manager->addTool("rviz/SetInitialPose");
//    tool_manager->setCurrentTool(current_tool);
//}
void qrviz::Set_goal_pose()
{
    rviz::Tool* current_tool = tool_manager->addTool("rviz/SetGoal");
    rviz::Property* pro = current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    tool_manager->setCurrentTool(current_tool);
}
