#ifndef QT_MQTT_COMM_H
#define QT_MQTT_COMM_H

#include <QObject>
#include <QString>
#include <QDebug>
#include <QtCore/QMap>
#include <QtMqtt/QMqttClient>
#include <QtMqtt/QMqttSubscription>
#include <QtMqtt/QMqttTopicName>
#include <QtMqtt/QMqttMessage>
#include "movement_control_single.h"

class qt_mqtt_comm : public QObject
{
    Q_OBJECT
public:
    explicit qt_mqtt_comm(QObject *parent = nullptr);
    ~qt_mqtt_comm();

    void initMqttClient(const QString &hostname, const quint16 port);
    void publishMessage(const QString &topic, const QString &message);
    void subscribeToTopic(const QString &topic);
    void unsubscribeFromTopic(const QString &topic);
    void connectToHost();
    void disconnectFromHost();
    const QString hostname() const;
    void setHostname(const QString &newHostname);
    int port() const;
    void setPort(const quint16 newPort);
signals:
    void messageReceived(const QString &topic, const QString &message);
private slots:
    void onConnected();
    void onDisconnected();
    void onMessageReceived(const QByteArray &message, const QMqttTopicName &topic);
    void onSubscribed(const QString &topic);
    void onUnsubscribed(const QString &topic);
    void onErrorOccurred(QMqttClient::ClientError error);
    QMqttClient::ClientState state() const;
    void setState(const QMqttClient::ClientState &newState);
private:
    QMqttClient *m_client;
    QMap<QString, QMqttSubscription*> m_subscriptions;
    MovementControl *movementControl;
};

#endif // QT_MQTT_COMM_H