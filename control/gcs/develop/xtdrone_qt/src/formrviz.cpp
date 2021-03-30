#include "../include/xtdrone_qt/formrviz.h"
#include "ui_formrviz.h"

FormRviz::FormRviz(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FormRviz)
{
    ui->setupUi(this);
}

FormRviz::~FormRviz()
{
    delete ui;
}


void FormRviz::on_button_image_cancel_clicked()
{
    this->hide();
}

void FormRviz::on_button_image_ok_clicked()
{
    if (ui->button_image_camera->isChecked())
    {
        emit rviz_to_main("image");
        this->hide();
    }
    else if(ui->button_image_tf->isChecked())
    {
        emit rviz_to_main("tf");
        this->hide();
    }
    else if(ui->button_image_laser->isChecked())
    {
        emit rviz_to_main("laser");
        this->hide();
    }
    else if(ui->button_image_addtopic->isChecked())
    {
        QString topic = ui->edit_addtopic->text();
        emit rviz_to_main(topic);
        this->hide();
    }
    else
        this->hide();


}
