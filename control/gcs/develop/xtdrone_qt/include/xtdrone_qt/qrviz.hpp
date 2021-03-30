#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <ros/ros.h>
#include <QVBoxLayout>

class qrviz
{
public:
    qrviz(QVBoxLayout* layout);
    void Set_FixedFrame(QString Frame_name);
    void Display_Grid(int Cell_count, QColor color, bool enable);
    void Display_TF(bool enable1, bool enable);
    void Display_LaserScan(QString laser_topic, bool enable);
    void Display_Image(QString Image_topic, bool enable, int num);
    void Display_Pose(QString pose_topic, QString pose_shape, QColor color, bool enable, int num);
    void Set_start_pose();
    void Set_goal_pose();
private:
    rviz::RenderPanel* render_panel;
    rviz::VisualizationManager* manager;
    rviz::Display* Grid = NULL;
    rviz::Display* Tf = NULL;
    rviz::Display* Laser = NULL;
    std::vector<rviz::Display*> Image;
    std::vector<rviz::Display*> Pose;
    rviz::ToolManager* tool_manager;
    int count_pose_num[20]={0};
    int count_image_num[20]={0};
};

#endif // QRVIZ_HPP
