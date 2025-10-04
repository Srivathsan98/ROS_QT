#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QIcon>
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>
#include <map>
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MovementControl;
}
QT_END_NAMESPACE

class MovementControl : public QMainWindow
{
    Q_OBJECT
public:
    explicit MovementControl(QWidget *parent = nullptr);
    ~MovementControl();
private:
    Ui::MovementControl *ui;
};
#endif // MOVEMENT_CONTROL_H
