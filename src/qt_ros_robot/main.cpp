#include "mainwindow.h"
#include "direction_menu.h"
#include "piemenu.h"
#include <QApplication>
#include "movement_control_single.h"
#include "qt_mqtt_comm.h"
#include <QTimer>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // MainWindow w;
    // w.show();
    // return a.exec();
    DirectionMenu menu;
    menu.setItems({"Forward", "Reverse", "Left", "Right", "Stop"});
    menu.show();

    PieMenu *piemenu;
    piemenu = new PieMenu(QIcon(QString("/home/pvsp/PER/drdc_hmi/res/icons/icon_camera_topview_active.svg")),200);
    piemenu->setSegment(Segment::TopSegment);
    piemenu->show();

#ifndef ROS_NODE
    qt_mqtt_comm* mqtt;
    MovementControl move(nullptr);
    move.show();
    mqtt = new qt_mqtt_comm(&move);
    mqtt->setHostname("localhost");
    mqtt->setPort(1883);
    qDebug() << "Connecting to MQTT broker at";
    mqtt->connectToHost();
#else
rclcpp::init(argc, argv);
    auto mqttNode = std::make_shared<MQTTNode>("tcp://localhost:1883", "ros2_node");

    MovementControl move(mqttNode);
    move.show();

    // Single-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mqttNode);

    // Use a Qt timer to spin ROS periodically
    QTimer rosTimer;
    QObject::connect(&rosTimer, &QTimer::timeout, [&executor]() {
        executor.spin_some();  // process pending ROS callbacks
    });
    rosTimer.start(10); // every 10 ms
#endif
    return a.exec();
}
