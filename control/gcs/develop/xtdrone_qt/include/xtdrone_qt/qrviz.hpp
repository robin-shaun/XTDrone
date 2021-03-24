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
private:
    rviz::RenderPanel* render_panel;
};

#endif // QRVIZ_HPP
