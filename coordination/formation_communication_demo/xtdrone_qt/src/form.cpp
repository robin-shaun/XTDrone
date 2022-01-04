#include "../include/xtdrone_qt/form.h"
#include "ui_form.h"

Form::Form(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Form)
{
    ui->setupUi(this);
}

Form::~Form()
{
    delete ui;
}

void Form::on_button_reset_clicked(bool)
{
    emit turn_to_main(true);
    this->hide();
}

void Form::on_button_ignore_clicked()
{
    this->hide();
}
