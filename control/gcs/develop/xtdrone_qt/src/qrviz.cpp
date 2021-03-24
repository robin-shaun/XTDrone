#include "../include/xtdrone_qt/qrviz.hpp"

qrviz::qrviz(QVBoxLayout* layout)
{
    // create rviz
    render_panel = new rviz::RenderPanel();
    layout->addWidget(render_panel);
}
