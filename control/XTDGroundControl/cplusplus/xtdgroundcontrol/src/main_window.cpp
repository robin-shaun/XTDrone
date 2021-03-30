/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/xtdgroundcontrol/main_window.hpp"
#include "QDebug"
//#include <sstream>
//#include <string>
#include <QColor>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xtdgroundcontrol {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc, argv)
//MainWindow::MainWindow(LISTINT multi_select, int *multi_num, QString *multi_type, QWidget *parent)
//    : QMainWindow(parent)
//    , qnode(multirotor_select, *multirotor_num, *multirotor_type)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
//    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    init_uisettings();
    setWindowIcon(QIcon("://images/xtd.ico"));
//    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    ui.show_widget_2->setCurrentIndex(0);

    /*********************
    ** QChart
    **********************/
    axisX->setRange(mapXY[0][0], mapXY[0][1]);
    axisX->setLabelsVisible(false);
    axisX->setGridLineVisible(false);
    axisX->setVisible(false);
    axisY->setRange(mapXY[0][2], mapXY[0][3]);
    axisY->setLabelsVisible(false);
    axisY->setGridLineVisible(false);
    axisY->setVisible(false);
    for (int i = 0; i < 10; i++)
        Seriesint(i);
    size_tab = ui.launch_info_text->size();
//    chart->createDefaultAxes();
    chart->legend()->hide();
    chart->setContentsMargins(0,0,0,0);
    chart->setMargins(QMargins(0,0,0,0));
    chart->setBackgroundRoundness(0);
    chartview->setChart(chart);
    chart->resize(size_tab);
    chartview->setRenderHint(QPainter::Antialiasing, true);
    ui.show_widget->addTab(chartview, "world");
    chartview->resize(size_tab);
    chartview->setContentsMargins(0,0,0,0);
    //settings of background
    widgetH = static_cast<int>(chart->plotArea().height());
    widgetW = static_cast<int>(chart->plotArea().width());
    qDebug()<<"widgetH"<<widgetH;
    qDebug()<<"widgetW"<<widgetW;

    QImage background("://images/indoor1.jpg");
    QImage background2 = background.scaled(widgetW,widgetH,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    QImage translated(size_tab.width(), size_tab.height(), QImage::Format_ARGB32);
//    translated.fill(Qt::white);
    QPainter painter(&translated);
//    TopLeft = chart->plotArea().topLeft();
    TopLeft = chart->plotArea().topLeft();
    painter.drawImage(TopLeft, background2);
    chart->setPlotAreaBackgroundBrush(translated);
    chart->setPlotAreaBackgroundVisible(true);
    /*********************
    ** init parameters
    **********************/
    ctrl_leader = false;
    cmd_vel_mask = false;
    formation_last = "waiting";
    /*********************
    ** init Rviz
    **********************/
    ui.treeWidget->setWindowTitle("Display");
//    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    QTreeWidgetItem* Global = new QTreeWidgetItem(QStringList()<<"Global Options");
    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);
    // FixFrame
    QTreeWidgetItem* Fixed_frame = new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    fixed_box = new QComboBox();
    fixed_box->addItem("map");
    fixed_box->addItem("base_footprint");
    fixed_box->addItem("laser");
    fixed_box->setMaximumWidth(120);
    fixed_box->setEditable(true);
    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));
    Global->addChild(Fixed_frame);
    ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);
    ui.treeWidget->header()->setMaximumSectionSize(150);
    ui.treeWidget->header()->setDefaultSectionSize(150);
    // Grid
    QTreeWidgetItem* Grid = new QTreeWidgetItem(QStringList()<<"Grid");
    QCheckBox* Grid_Check = new QCheckBox();
    connect(Grid_Check, SIGNAL(stateChanged(int)), this, SLOT(slot_display_grid(int)));
    ui.treeWidget->addTopLevelItem(Grid);
    ui.treeWidget->setItemWidget(Grid,1,Grid_Check);
    Grid->setExpanded(true);
    QTreeWidgetItem* Cell_count = new QTreeWidgetItem(QStringList()<<"Cell Count");
    Grid->addChild(Cell_count);
    Cell_count_box = new QSpinBox();
    Cell_count_box->setValue(15);
    Cell_count_box->setMaximumWidth(120);
    ui.treeWidget->setItemWidget(Cell_count,1,Cell_count_box);
    QTreeWidgetItem* Grid_color = new QTreeWidgetItem(QStringList()<<"Color");
    Grid->addChild(Grid_color);
    Grid_color_box = new QComboBox();
    Grid_color_box->addItem("160;160;160");
    Grid_color_box->setEditable(true);
    Grid_color_box->setMaximumWidth(120);
    ui.treeWidget->setItemWidget(Grid_color,1,Grid_color_box);
    signalMapper_image = new QSignalMapper(this);
    signalMapper_pose = new QSignalMapper(this);

}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool) {
//    qDebug()<<"init!!!";
//    if ( !qnode.init() ) {   //toStdString()
//        showNoMasterMessage();
//    } else {
//        ui.button_run->setEnabled(false);
//    }
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "xtdgroundcontrol");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    connect(ui.button_run,SIGNAL(clicked(bool)),this,SLOT(slot_btn_run_click(bool)));
    connect(ui.button_stop,SIGNAL(clicked(bool)),this,SLOT(slot_init(bool)));
    connect(ui.button_control,SIGNAL(clicked(bool)),this,SLOT(slot_btn_control_click(bool)));
    connect(ui.box_go_and_back_2,SIGNAL(valueChanged(double)),this,SLOT(slot_box_valuechange(double)));
    connect(ui.box_up_and_down_2,SIGNAL(valueChanged(double)),this,SLOT(slot_box_valuechange(double)));
    connect(ui.box_left_and_right_2,SIGNAL(valueChanged(double)),this,SLOT(slot_box_valuechange(double)));
    connect(ui.box_orientation,SIGNAL(valueChanged(double)),this,SLOT(slot_box_valuechange(double)));
    connect(ui.box_command,SIGNAL(currentTextChanged(const QString)),this,SLOT(box_command_valuechange(const QString)));
    connect(ui.box_formation,SIGNAL(currentTextChanged(const QString)),this,SLOT(box_command_valuechange(const QString)));
    //connect(ui.comboBox_maps,SIGNAL(currentIndexChanged(const QString)),this,SLOT(slot_box_maps_valuechange(const QString)));
    connect(ui.checkBox_iris,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_solo,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_plane,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_rotor,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_typhoon,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_quadplane,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_tiltrotor,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.checkBox_tailsitter,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_get_uav_control(int)));
    connect(ui.button_connect,SIGNAL(clicked(bool)),this,SLOT(slot_btn_connect_click(bool)));
    connect(ui.comboBox_maps,SIGNAL(currentTextChanged(const QString)),this,SLOT(slot_box_maps_change(const QString)));
    connect(&qnode, SIGNAL(uavposition(float, float)),this,SLOT(slot_update_plot(float, float)));
    connect(ui.button_add, SIGNAL(clicked(bool)),this,SLOT(slot_btn_add_click(bool)));
//    connect(ui.button_estimate, SIGNAL(clicked(bool)),this,SLOT(slot_btn_estimate_click(bool)));
    connect(ui.button_goal, SIGNAL(clicked(bool)),this,SLOT(slot_btn_goal_click(bool)));
    connect(&qnode, SIGNAL(rvizsetgoal()),this,SLOT(slot_rviz_control()));
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "xtdgroundcontrol");
    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::Seriesint(int ch)
{
    seriesList[ch] = new QtCharts::QLineSeries();
    chart->addSeries(seriesList[ch]);
    randnum = rand()%(255-16+1)+16;
    color.setRed(randnum);
    randnum = rand()%(255-16+1)+16;
    color.setGreen(randnum);
    randnum = rand()%(255-16+1)+16;
    color.setBlue(randnum);
    color_plot.push_back(color);
    seriesList[ch]->setPen(QPen(color,2,Qt::SolidLine));
    chart->setAxisX(axisX, seriesList[ch]);
    chart->setAxisY(axisY, seriesList[ch]);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::init_uisettings()
{
    ui.button_run->setChecked(false);
    ui.button_stop->setChecked(false);
    ui.button_control->setChecked(false);
    ui.button_run->setEnabled(true);
    ui.button_stop->setEnabled(false);
    ui.button_control->setEnabled(false);
    ui.box_go_and_back_2->setReadOnly(true);
    ui.box_up_and_down_2->setReadOnly(true);
    ui.box_left_and_right_2->setReadOnly(true);
    ui.box_orientation->setReadOnly(true);
    ui.box_formation->setVisible(false);
    ui.checkBox_iris->setCheckable(false);
    ui.checkBox_solo->setCheckable(false);
    ui.checkBox_plane->setCheckable(false);
    ui.checkBox_rotor->setCheckable(false);
    ui.checkBox_typhoon->setCheckable(false);
    ui.checkBox_quadplane->setCheckable(false);
    ui.checkBox_tiltrotor->setCheckable(false);
    ui.checkBox_tailsitter->setCheckable(false);
    ui.button_goal->setEnabled(false);

//    ui.button_add->setChecked(false);
//    ui.button_add->setEnabled(false);

    once_flag = 0;
    twice_flag = 0;
    ui.box_go_and_back_2->setProperty("value", 0.0);
    ui.box_up_and_down_2->setProperty("value", 0.0);
    ui.box_left_and_right_2->setProperty("value", 0.0);
    ui.box_orientation->setProperty("value", 0.0);

}

void MainWindow::slot_init(bool)
{
    qDebug()<<"init?";
    init_uisettings();
    qnode.stop_control(true);
}
void MainWindow::slot_btn_connect_click(bool)
{
    // auto run:
    QString launch_text = ui.text_launch_2->toPlainText();
    if (launch_text != "")
    {
        launch_cmd = new QProcess;
        launch_cmd->start("bash");
        launch_cmd->write(ui.text_launch_2->toPlainText().toLocal8Bit()+'\n');
        connect(launch_cmd,SIGNAL(readyReadStandardError()),this, SLOT(slot_quick_output()));
        connect(launch_cmd,SIGNAL(readyReadStandardOutput()),this, SLOT(slot_quick_output()));
    }

}

void MainWindow::slot_btn_run_click(bool)
{
    //self.control_type = str(self.comboBox_controltype.currentText())
//    Form* f=new Form;
//    f->show();
//    connect(f,SIGNAL(turn_to_main(bool)),this,SLOT(slot_init(bool)));
    once_flag +=1;
    bool run_flag = true;
    bool formation_flag = true;
    map = QString(ui.comboBox_maps->currentText());
    multirotor_select.clear();
    multi_type.clear();

    // get setting values of multi-uav
    if (once_flag==1)
    {
        control_type = QString(ui.comboBox_controltype->currentText());
        for (int i = 0; i < multi_type_num; i++)
        {
            if (i == 0)
            {
                multirotor_num[i] = int(ui.box_iris_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_iris->setCheckable(true);
                }
            }
            else if (i == 1)
            {
                multirotor_num[i] = int(ui.box_typhoon_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_typhoon->setCheckable(true);
                }
            }
            else if (i == 2)
            {
                multirotor_num[i] = int(ui.box_solo_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_solo->setCheckable(true);
                }
            }
            else if (i == 3)
            {
                multirotor_num[i] = int(ui.box_tailsitter_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_tailsitter->setCheckable(true);
                }
            }
            else if (i == 4)
            {
                multirotor_num[i] = int(ui.box_quadplane_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_quadplane->setCheckable(true);
                }
            }
            else if (i == 5)
            {
                multirotor_num[i] = int(ui.box_plane_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_plane->setCheckable(true);
                }
            }
            else if (i == 6)
            {
                multirotor_num[i] = int(ui.box_tiltrotor_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_tiltrotor->setCheckable(true);
                }
            }
            else
            {
                multirotor_num[i] = int(ui.box_rotor_num->value());
                if (multirotor_num[i] > 0.5)
                {
                    multirotor_select.push_back(i);
                    ui.checkBox_rotor->setCheckable(true);
                }
            }
            if ((multirotor_num[i] == 6) || (multirotor_num[i] == 9) || (multirotor_num[i] == 18))
            {
                if (formation_flag)
                {
                    ui.box_formation->setVisible(true);
                    multirotor_formation_num = multirotor_num[i];
                    formation_flag = false;
                }
                else
                {
                    run_flag = false;
                    Form* f=new Form;
                    f->show();
                    connect(f,SIGNAL(turn_to_main()),this,SLOT(slot_init()));
                }
            }

        }
        if (run_flag && once_flag==1)
        {
            multi_num = 0;
            int select_num = multirotor_select.size();
            qDebug()<<"select_num:"<<select_num;
            for (int j = 0; j < select_num ; j++)
            {
                multi_num = multi_num + multirotor_num[multirotor_select[j]];
                for (int id = 0; id < multirotor_num[multirotor_select[j]] ; id++)
                    multi_type.push_back(multirotor_type[multirotor_select[j]].toStdString());
//                qDebug()<<"multirotor_type"<<multi_type[j].c_str();
            }
//            qDebug()<<"num:"<<multi_num;
//            qDebug()<<"multirotor_select"<<multirotor_select;
//            for (int ii = 0; ii< 8; ii++)
//                qDebug()<<"multirotor_num"<< multirotor_num[ii];
            qDebug()<<"init!!!";

            if ( !qnode.init(multirotor_select, multirotor_num, multi_type,control_type.toStdString())) {   //toStdString()
                showNoMasterMessage();
                ui.treeWidget->setEnabled(false);
            } else {
                ui.button_run->setEnabled(false);
                ui.button_control->setEnabled(true);

                my_rviz = new qrviz(ui.verticalLayout_rviz);
                ui.treeWidget->setEnabled(true);
//                ui.button_add->setEnabled(true);
            }
        }

    }

    //qDebug()<<multirotor_select;
    //qDebug()<<multirotor_num[0];
}

//void MainWindow::slot_btn_stop_click(bool)
//{
//    init_uisettings();
//}

void MainWindow::slot_btn_control_click(bool)
{
    twice_flag += 1;
    qnode.start_control(true);
    if (twice_flag == 1)
    {
        ui.box_go_and_back_2->setReadOnly(false);
        ui.box_up_and_down_2->setReadOnly(false);
        ui.box_left_and_right_2->setReadOnly(false);
        ui.box_orientation->setReadOnly(false);
        ui.button_stop->setEnabled(true);
        ui.button_goal->setEnabled(true);
    }

}

void MainWindow::slot_quick_output()
{
    ui.launch_info_text->append("<font color=\"#FF0000\">"+launch_cmd->readAllStandardError()+"</font>");
    ui.launch_info_text->append("<font color=\"#000000\">"+launch_cmd->readAllStandardOutput()+"</font>");
}
void MainWindow::slot_box_valuechange(double)
{
    qDebug()<<"change!!!";
    double forward_change = ui.box_go_and_back_2->value();
    double upward_change = ui.box_up_and_down_2->value();
    double leftward_change = ui.box_left_and_right_2->value();
    double orientation_change = ui.box_orientation->value();
    ui.box_command->setCurrentIndex(0);
    std::string cmd3 = "";
    cmd3 = cmd2.toStdString();
    qnode.set_cmd(forward_change, upward_change, leftward_change, orientation_change, ctrl_leader, cmd3, cmd_vel_mask);
}

void MainWindow::box_command_valuechange(const QString)
{
    qDebug()<<"change!!!";
    double forward_change = ui.box_go_and_back_2->value();
    double upward_change = ui.box_up_and_down_2->value();
    double leftward_change = ui.box_left_and_right_2->value();
    double orientation_change = ui.box_orientation->value();
    QString cmd_mid = ui.box_command->currentText();
    QString formation_mid = ui.box_formation->currentText();
    if (cmd_mid == "control all")
    {
        cmd2 = "";
        ctrl_leader = false;
    }
    else if (cmd_mid == "arm")
    {
        cmd2 = "ARM";
    }
    else if (cmd_mid == "disarm")
    {
        cmd2 = "DISARM";
    }
    else if (cmd_mid == "return home")
    {
        cmd2 = "AUTO.RTL";
    }
    else if (cmd_mid == "take off")
    {
        cmd2 = "AUTO.TAKEOFF";
    }
    else if (cmd_mid == "land")
    {
        cmd2 = "AUTO.LAND";
    }
    else if (cmd_mid == "offboard")
    {
        cmd2 = "OFFBOARD";
    }
    else if (cmd_mid == "hover")
    {
        cmd2 = "HOVER";
        cmd_vel_mask = false;
        ui.box_go_and_back_2->setProperty("value", 0.0);
        ui.box_up_and_down_2->setProperty("value", 0.0);
        ui.box_left_and_right_2->setProperty("value", 0.0);
        ui.box_orientation->setProperty("value", 0.0);
        forward_change = 0.0;
        upward_change = 0.0;
        leftward_change = 0.0;
        orientation_change = 0.0;
    }
    else if (cmd_mid == "control the leader")
    {
        cmd2 = "";
        ctrl_leader = true;
    }
    else
        cmd2 = "";

    if (formation_last != formation_mid)
    {
        cmd_vel_mask = true;
        formation_last = formation_mid;
        cmd2 = MainWindow::box_formation_valuechange(formation_mid);
    }
    std::string cmd3;
    cmd3 = cmd2.toStdString();
    qnode.set_cmd(forward_change, upward_change, leftward_change, orientation_change, ctrl_leader, cmd3, cmd_vel_mask);
}

QString MainWindow::box_formation_valuechange(const QString formation_mid)
{
    if (multirotor_formation_num == 6)
    {
        if (formation_mid == "formation 1")
        {
            formation2 = "T";
        }
        else if (formation_mid == "formation 2")
        {
            formation2 = "diamond";
        }
        else if (formation_mid == "formation 3")
        {
            formation2 = "triangle";
        }
        else
        {
            formation2 = "waiting";
        }
    }
    else if (multirotor_formation_num == 9)
    {
        if (formation_mid == "formation 1")
        {
            formation2 = "cube";
        }
        else if (formation_mid == "formation 2")
        {
            formation2 = "pyramid";
        }
        else if (formation_mid == "formation 3")
        {
            formation2 = "triangle";
        }
        else
        {
            formation2 = "waiting";
        }
    }
    else
    {
        if (formation_mid == "formation 1")
        {
            formation2 = "cuboid";
        }
        else if (formation_mid == "formation 2")
        {
            formation2 = "sphere";
        }
        else if (formation_mid == "formation 3")
        {
            formation2 = "diamond";
        }
        else
        {
            formation2 = "waiting";
        }
    }
    return formation2;
}

void MainWindow::slot_checkbox_get_uav_control(int)
{
    multirotor_get_control.clear();
    for (int i = 0; i<multirotor_num[0] ;i++)
    {
        if (ui.checkBox_iris->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[1] ;i++)
    {
        if (ui.checkBox_typhoon->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[2] ;i++)
    {
        if (ui.checkBox_solo->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[3] ;i++)
    {
        if (ui.checkBox_tailsitter->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[4] ;i++)
    {
        if (ui.checkBox_quadplane->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[5] ;i++)
    {
        if (ui.checkBox_plane->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[6] ;i++)
    {
        if (ui.checkBox_tiltrotor->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    for (int i = 0; i<multirotor_num[7] ;i++)
    {
        if (ui.checkBox_rotor->isChecked())
            multirotor_get_control.push_back(1);
        else
            multirotor_get_control.push_back(0);
    }
    qDebug()<<"control::::"<<multirotor_get_control;
    qnode.get_uav_control(multirotor_get_control);
}

void MainWindow::slot_box_maps_change(const QString value)
{
    QString map_address;
    map = "://images/"+value+".jpg";
    if (value == "indoor1")
    {
        axisX->setRange(mapXY[0][0], mapXY[0][1]);
        axisY->setRange(mapXY[0][2], mapXY[0][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(-6.1, 0) << QPointF(3.7, -8.4);
    }
    else if(value == "indoor2")
    {
        axisX->setRange(mapXY[1][0], mapXY[1][1]);
        axisY->setRange(mapXY[1][2], mapXY[1][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(-6.1, 0) << QPointF(3.7, -8.4);
    }
    else if(value == "indoor3")
    {
        axisX->setRange(mapXY[2][0], mapXY[2][1]);
        axisY->setRange(mapXY[2][2], mapXY[2][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(-6.1, 0) << QPointF(3.7, -8.4);
    }
    else if(value == "outdoor1")
    {
        axisX->setRange(mapXY[3][0], mapXY[3][1]);
        axisY->setRange(mapXY[3][2], mapXY[3][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(50, -1.2);
    }
    else if(value == "outdoor2")
    {
        axisX->setRange(mapXY[4][0], mapXY[4][1]);
        axisY->setRange(mapXY[4][2], mapXY[4][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(-6.1, 0) << QPointF(3.7, -8.4);
    }
    else if(value == "outdoor3")
    {
        axisX->setRange(mapXY[5][0], mapXY[5][1]);
        axisY->setRange(mapXY[5][2], mapXY[5][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(-6.1, 0) << QPointF(3.7, -8.4);
    }
    else if(value == "robocup")
    {
        axisX->setRange(mapXY[6][0], mapXY[6][1]);
        axisY->setRange(mapXY[6][2], mapXY[6][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(50, -1.2);
    }
    else
    {
        axisX->setRange(mapXY[7][0], mapXY[7][1]);
        axisY->setRange(mapXY[7][2], mapXY[7][3]);
//        chart->removeSeries(series);
//        series->clear();
//        *series << QPointF(0, 0) << QPointF(-6.1, 0) << QPointF(3.7, -8.4);
    }
    widgetH = static_cast<int>(chart->plotArea().height());
    widgetW = static_cast<int>(chart->plotArea().width());
    QImage background(map);
    QImage background2 = background.scaled(widgetW,widgetH,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
//    background2.save("123.jpg");
//    ui.show_widget->setStyleSheet(R"(QGraphicsView{ background-image:url(123.jpg);})");
    QImage translated(size_tab.width(), size_tab.height(), QImage::Format_ARGB32);
    translated.fill(Qt::white);
    QPainter painter(&translated);
    painter.drawImage(TopLeft, background2);
    chart->setPlotAreaBackgroundBrush(translated);
}
void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::slot_update_plot(float x_position, float y_position)
{
//    qDebug()<<"i got it";
    if (plot_count >= multi_num)
        plot_count = 0;
    seriesList[plot_count]->append(x_position, y_position);
//    if (plot_count==1 ||plot_count ==5)
//    {
//        qDebug()<<"series"<<plot_count<<":"<<x_position;
//        qDebug()<<"series"<<plot_count<<":"<<y_position;
//    }
    plot_count ++;
}

void MainWindow::slot_treewidget_value_change(QString)
{
    my_rviz->Set_FixedFrame(fixed_box->currentText());
}

void MainWindow::slot_display_grid(int state)
{
    bool enable=state>1?true:false;
    QStringList cut_color = Grid_color_box->currentText().split(";");
    QColor color = QColor(cut_color[0].toInt(),cut_color[1].toInt(),cut_color[2].toInt());
    my_rviz->Display_Grid(Cell_count_box->text().toInt(), color, enable);
}

void MainWindow::slot_display_tf(int state)
{
    bool enable=state>1?true:false;
    int state1 = Tf_showname_Check->checkState();
    bool enable1=state1>1?true:false;
    my_rviz->Display_TF(enable1, enable);
}

void MainWindow::slot_display_Laser(int state)
{
    bool enable=state>1?true:false;
    my_rviz->Display_LaserScan(Laser_topic_box->currentText(),enable);
}

void MainWindow::slot_display_Image(int num)
{
    bool enable = Image_Check[num]->checkState()>1?true:false;
    my_rviz->Display_Image(Image_topic_box[num]->currentText(),enable, num);
}

void MainWindow::slot_display_Pose(int num)
{
    bool enable = Pose_Check[num]->checkState()>1?true:false;
    QStringList cut_color = Pose_color_box[num]->currentText().split(";");
    QColor color = QColor(cut_color[0].toInt(),cut_color[1].toInt(),cut_color[2].toInt());
    my_rviz->Display_Pose(Pose_topic_box[num]->currentText(),Pose_shape_box[num]->currentText(),color,enable,num);
    qDebug()<<"test!!!";
}

void MainWindow::slot_btn_add_click(bool)
{
    FormRviz* frviz=new FormRviz;
    frviz->show();
    connect(frviz,SIGNAL(rviz_to_main(QString)),this,SLOT(slot_addrviz(QString)));
}

void MainWindow::slot_addrviz(QString value)
{
    qDebug()<<value;
    if (value == "tf")
    {
        //TF
        QTreeWidgetItem* TF = new QTreeWidgetItem(QStringList()<<"TF");
        QCheckBox* TF_Check = new QCheckBox();
        connect(TF_Check, SIGNAL(stateChanged(int)), this, SLOT(slot_display_tf(int)));
        ui.treeWidget->addTopLevelItem(TF);
        ui.treeWidget->setItemWidget(TF,1,TF_Check);
        QTreeWidgetItem* Tf_showname = new QTreeWidgetItem(QStringList()<<"Show Names");
        TF->addChild(Tf_showname);
        Tf_showname_Check = new QCheckBox();
        ui.treeWidget->setItemWidget(Tf_showname,1,Tf_showname_Check);
        Tf_showname_Check->setCheckState(Qt::CheckState(2));
    }
    else if (value == "laser")
    {
        //Laser
        QTreeWidgetItem* Laser = new QTreeWidgetItem(QStringList()<<"Laser Scan");
        QCheckBox* Laser_Check = new QCheckBox();
        connect(Laser_Check, SIGNAL(stateChanged(int)), this, SLOT(slot_display_Laser(int)));
        ui.treeWidget->addTopLevelItem(Laser);
        ui.treeWidget->setItemWidget(Laser,1,Laser_Check);
        QTreeWidgetItem* Laser_topic = new QTreeWidgetItem(QStringList()<<"Topic");
        Laser->addChild(Laser_topic);
        Laser_topic_box = new QComboBox();
        Laser_topic_box->addItem("/scan");
        Laser_topic_box->setEditable(true);
        Laser_topic_box->setMaximumWidth(120);
        ui.treeWidget->setItemWidget(Laser_topic,1,Laser_topic_box);
    }
    else if (value == "image")
    {
        //Image
        QTreeWidgetItem* Image = new QTreeWidgetItem(QStringList()<<"Image");
        Image_Check.push_back(new QCheckBox());
        connect(Image_Check[image_num], SIGNAL(stateChanged(int)), signalMapper_image, SLOT(map()));
        signalMapper_image->setMapping(Image_Check[image_num],image_num);
        connect(signalMapper_image,SIGNAL(mapped(int)),this,SLOT(slot_display_Image(int)));
        ui.treeWidget->addTopLevelItem(Image);
        ui.treeWidget->setItemWidget(Image,1,Image_Check[image_num]);
        QTreeWidgetItem* Image_topic = new QTreeWidgetItem(QStringList()<<"Topic");
        Image->addChild(Image_topic);
        Image_topic_box.push_back( new QComboBox());
        Image_topic_box[image_num]->addItem("");
        Image_topic_box[image_num]->setEditable(true);
        Image_topic_box[image_num]->setMaximumWidth(120);
        ui.treeWidget->setItemWidget(Image_topic,1,Image_topic_box[image_num]);
        image_num++;
    }
    else
    {
        value.remove(QChar('\n'), Qt::CaseInsensitive);
        //Add topic
        QTreeWidgetItem* Pose = new QTreeWidgetItem(QStringList()<<"Pose");
        Pose_Check.push_back(new QCheckBox());
        connect(Pose_Check[pose_num], SIGNAL(stateChanged(int)), signalMapper_pose, SLOT(map()));
        signalMapper_pose->setMapping(Pose_Check[pose_num],pose_num);
        connect(signalMapper_pose,SIGNAL(mapped(int)),this,SLOT(slot_display_Pose(int)));
        ui.treeWidget->addTopLevelItem(Pose);
        ui.treeWidget->setItemWidget(Pose,1,Pose_Check[pose_num]);
        QTreeWidgetItem* Pose_topic = new QTreeWidgetItem(QStringList()<<"Topic");
        Pose->addChild(Pose_topic);
        Pose_topic_box.push_back(new QComboBox());
        Pose_topic_box[pose_num]->addItem(value);
        Pose_topic_box[pose_num]->setEditable(true);
        Pose_topic_box[pose_num]->setMaximumWidth(200);
        ui.treeWidget->setItemWidget(Pose_topic,1,Pose_topic_box[pose_num]);
        QTreeWidgetItem* Pose_color = new QTreeWidgetItem(QStringList()<<"Color");
        Pose->addChild(Pose_color);
        Pose_color_box.push_back(new QComboBox());
        Pose_color_box[pose_num]->addItem("160;160;160");
        Pose_color_box[pose_num]->setEditable(true);
        Pose_color_box[pose_num]->setMaximumWidth(120);
        ui.treeWidget->setItemWidget(Pose_color,1,Pose_color_box[pose_num]);
        QTreeWidgetItem* Pose_shape = new QTreeWidgetItem(QStringList()<<"Shape");
        Pose->addChild(Pose_shape);
        Pose_shape_box.push_back(new QComboBox());
        Pose_shape_box[pose_num]->addItem("Axes");
        Pose_shape_box[pose_num]->addItem("Arrow");
        Pose_shape_box[pose_num]->setEditable(true);
        Pose_shape_box[pose_num]->setMaximumWidth(120);
        ui.treeWidget->setItemWidget(Pose_shape,1,Pose_shape_box[pose_num]);
        pose_num++;
    }
}

//void MainWindow::slot_btn_estimate_click(bool)
//{
//    my_rviz->Set_start_pose();
//}

void MainWindow::slot_btn_goal_click(bool)
{
    my_rviz->Set_goal_pose();
}
void MainWindow::slot_rviz_control()
{
    ui.box_go_and_back_2->setProperty("value", 0.0);
    ui.box_up_and_down_2->setProperty("value", 0.0);
    ui.box_left_and_right_2->setProperty("value", 0.0);
    ui.box_orientation->setProperty("value", 0.0);
}
}  // namespace xtdgroundcontrol
