#include "movement_control_single.h"
#include "ui_movement_control_single.h"
MovementControl::MovementControl(QWidget *parent)
    : QMainWindow{parent}
    , ui(new Ui::MovementControl)
{
    ui->setupUi(this);
    initMovementSignalSlotConnection();
}
MovementControl::~MovementControl()
{
    delete ui;
}

void MovementControl::initMovementSignalSlotConnection()
{
    connect(ui->front, &QPushButton::clicked, this, &MovementControl::onForwardButtonClicked);
    connect(ui->left, &QPushButton::clicked, this, &MovementControl::onLeftButtonClicked);
    connect(ui->right, &QPushButton::clicked, this, &MovementControl::onRightButtonClicked);
    connect(ui->rear, &QPushButton::clicked, this, &MovementControl::onReverseButtonClicked);
    connect(ui->stop, &QPushButton::clicked, this, &MovementControl::onStopButtonClicked);
}

void MovementControl::onForwardButtonClicked()
{
    emit moveforward();
    qDebug() << "front signal emitted";
}

void MovementControl::onLeftButtonClicked()
{
    emit moveleft();
    qDebug() << "left signal emitted";
}

void MovementControl::onRightButtonClicked()
{
    emit moveright();
    qDebug() << "right signal emitted";
}

void MovementControl::onReverseButtonClicked()
{
    emit movereverse();
    qDebug() << "reverse signal emitted";
}

void MovementControl::onStopButtonClicked()
{
    emit stopmovement();
    qDebug() << "stop signal emitted";
}