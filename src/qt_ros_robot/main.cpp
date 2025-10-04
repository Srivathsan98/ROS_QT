#include "mainwindow.h"
#include "direction_menu.h"
#include "piemenu.h"
#include <QApplication>
#include "movement_control_single.h"
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

    return a.exec();
}
