#include "movement_control_single.h"
#include "ui_movement_control_single.h"
MovementControl::MovementControl(QWidget *parent)
    : QMainWindow{parent}
    , ui(new Ui::MovementControl)
{
    ui->setupUi(this);
}
MovementControl::~MovementControl()
{
    delete ui;
}



