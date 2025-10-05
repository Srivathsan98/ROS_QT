#include "mainwindow.h"
#include "direction_menu.h"
#include "piemenu.h"
#include <QApplication>
#include "movement_control_single.h"
#include "qt_mqtt_comm.h"

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

    MovementControl move;
    move.show();

    qt_mqtt_comm* mqtt;
    mqtt = new qt_mqtt_comm(&move);
    mqtt->setHostname("localhost");
    mqtt->setPort(1883);
    qDebug() << "Connecting to MQTT broker at";
    mqtt->connectToHost();

    return a.exec();
}
