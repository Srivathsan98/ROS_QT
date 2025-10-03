#include "mainwindow.h"
#include "direction_menu.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // MainWindow w;
    // w.show();
    // return a.exec();
    DirectionMenu menu;
    menu.setItems({"Forward", "Reverse", "Left", "Right", "Stop"});
    menu.show();

    return a.exec();
}
