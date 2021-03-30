/**
 * @file /include/xtdrone_qt/main_window.hpp
 *
 * @brief Qt based gui for xtdrone_qt.
 *
 * @date November 2010
 **/
#ifndef xtdrone_qt_MAIN_WINDOW_H
#define xtdrone_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "form.h"
#include "formrviz.h"
#include "qrviz.hpp"
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QProcess>
#include <QColor>
#include <QComboBox>
#include <QSpinBox>
#include <QSignalMapper>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace xtdrone_qt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT


public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
//    MainWindow(LISTINT multirotor_select, int *multirotor_num, QString *multirotor_type QWidget *parent = 0);
    ~MainWindow();
    static const int multi_type_num = 8;
    QString multirotor_type[multi_type_num] = {"iris", "typhoon_h480", "solo", "tailsitter", "quadplane", "plane", "tiltrotor", "ugv"};
    int multirotor_num[multi_type_num] = {1, 0, 0, 0, 0, 0, 0, 0};
    double mapXY[multi_type_num][4] = {{-20.0, 21.5, -13.3, 10.7},{-10.5, 17.2, -12.4, 3.5},{-11.54, 21.34, -2.2, 16.8},{-48, 152, -57.09, 59.09}, {-2184, 2980, -1500, 1500},{-1192.9, 648.9, -375, 695},{-48, 152, -57.09, 59.09}, {-100,100,-100,100}};
    int multirotor_formation_num = 0;
    QString control_type = "vel";
    QString task = "Flying";
    QString map = "indoor1";
    QString formation2 = "formation1";
    QString cmd2 = "controll all";
    int once_flag = 0;
    int twice_flag = 0;
    typedef QList<int> LISTINT;
    typedef QList<std::string> LISTSTR;
    LISTINT multirotor_select;
    LISTINT multirotor_get_control;
    LISTSTR multi_type;
    int multi_num;
    QProcess *launch_cmd;
    bool ctrl_leader;
    bool cmd_vel_mask;
    QString formation_last;
    int ViewW = 882; //(1/2.5)
    int ViewH = 513; //(1/2.5)
    double widgetW;
    double widgetH;
//    typedef QList<QtCharts::QLineSeries*> LISTSERIES;
//    LISTSERIES seriesList;
    std::vector<QColor> color_plot;
    QtCharts::QLineSeries *series = new QtCharts::QLineSeries();
    QtCharts::QLineSeries* seriesList[10];
    QtCharts::QChart* chart = new QtCharts::QChart();
    QtCharts::QValueAxis *axisX = new QtCharts::QValueAxis;
    QtCharts::QValueAxis *axisY = new QtCharts::QValueAxis;
    QtCharts::QChartView* chartview = new QtCharts::QChartView();
    QPointF TopLeft;
    QSize size_tab;
    double zoom;
    int plot_count = 0;
    int randnum;
    QColor color;
    QComboBox* fixed_box;
    QSpinBox* Cell_count_box;
    QComboBox* Grid_color_box;
    QComboBox* Laser_topic_box;
    std::vector<QCheckBox*> Pose_Check;
    std::vector<QCheckBox*> Image_Check;
    std::vector<QComboBox*> Image_topic_box;
    std::vector<QComboBox*> Pose_topic_box;
    std::vector<QComboBox*> Pose_color_box;
    std::vector<QComboBox*> Pose_shape_box;
    QCheckBox* Tf_showname_Check;
    int pose_num=0;
    int image_num=0;
    QSignalMapper* signalMapper_image;
    QSignalMapper* signalMapper_pose;


    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing
    void Seriesint(int);
    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
//    void on_button_connect_clicked(bool check );
    void slot_btn_run_click(bool);
    void slot_btn_control_click(bool);
    void slot_btn_connect_click(bool);
//    void slot_box_go_and_back_2_valuechange(double);
//    void slot_box_up_and_down_2_valuechange(double);
//    void slot_box_left_and_right_2_valuechange(double);
    void slot_box_valuechange(double);
    void box_command_valuechange(const QString);
    QString box_formation_valuechange(const QString);
    //void slot_box_maps_valuechange(const QString);
    void slot_checkbox_get_uav_control(int);
    void slot_init(bool);
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_quick_output();
    void slot_box_maps_change(const QString);
    void slot_update_plot(float, float);
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_Laser(int);
    void slot_display_Image(int);
    void slot_display_Pose(int);
    void slot_btn_add_click(bool);
    void slot_addrviz(QString);
    void slot_btn_estimate_click(bool);
    void slot_btn_goal_click(bool);
    void slot_rviz_control();
private:
    void init_uisettings();
    Ui::MainWindowDesign ui;
    QNode qnode;
    qrviz* my_rviz;
};

}  // namespace xtdrone_qt

#endif // xtdrone_qt_MAIN_WINDOW_H
