#ifndef FORMRVIZ_H
#define FORMRVIZ_H

#include <QWidget>

namespace Ui {
class FormRviz;
}

class FormRviz : public QWidget
{
    Q_OBJECT

public:
    explicit FormRviz(QWidget *parent = nullptr);
    ~FormRviz();

private slots:
    void on_button_image_cancel_clicked();
    void on_button_image_ok_clicked();

signals:
    void rviz_to_main(QString);

private:
    Ui::FormRviz *ui;
};

#endif // FORMRVIZ_H
