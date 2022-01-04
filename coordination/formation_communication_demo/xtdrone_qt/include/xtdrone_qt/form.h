#ifndef xtdrone_qt_FORM_H
#define xtdrone_qt_FORM_H

#include <QWidget>

namespace Ui {
class Form;
}

class Form : public QWidget
{
    Q_OBJECT

public:
    explicit Form(QWidget *parent = nullptr);
    ~Form();

private slots:
    void on_button_reset_clicked(bool);
    void on_button_ignore_clicked();

signals:
    void turn_to_main(bool);

private:
    Ui::Form *ui;
};

#endif // FORM_H
