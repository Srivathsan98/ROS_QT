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
#include <QDebug>
#include <memory>
#ifdef ROS_NODE
#include <rclcpp/rclcpp.hpp>
#include "mqtt_ros_node.h"
#endif

QT_BEGIN_NAMESPACE
namespace Ui {
class MovementControl;
}
QT_END_NAMESPACE

class MovementControl : public QMainWindow
{
    Q_OBJECT
public:
    explicit MovementControl(std::shared_ptr<MQTTNode> mqttNode, QWidget *parent = nullptr);
    ~MovementControl();
private:
    Ui::MovementControl *ui;
    std::shared_ptr<MQTTNode> mqttNode;

    void initMovementSignalSlotConnection();

signals:
    void moveforward();
    void moveleft();
    void moveright();
    void movereverse();
    void stopmovement();

private slots:
    void onForwardButtonClicked();
    void onLeftButtonClicked();
    void onRightButtonClicked();
    void onReverseButtonClicked();
    void onStopButtonClicked();
};
#endif // MOVEMENT_CONTROL_H
