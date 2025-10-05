#include "qt_mqtt_comm.h"

qt_mqtt_comm::qt_mqtt_comm(QObject *parent)
    : QObject(parent)
    , m_client(new QMqttClient(this))
    , movementControl(new MovementControl())
{
    connect(m_client, &QMqttClient::connected, this, &qt_mqtt_comm::onConnected);
    connect(m_client, &QMqttClient::disconnected, this, &qt_mqtt_comm::onDisconnected);
    connect(m_client, &QMqttClient::messageReceived, this, &qt_mqtt_comm::onMessageReceived);
    connect(m_client, &QMqttClient::errorChanged, this, &qt_mqtt_comm::onErrorOccurred);

    connect(movementControl, &MovementControl::moveforward, [this]() { publishMessage("robot/move", "forward");});
    connect(movementControl, &MovementControl::moveleft, [this]() { publishMessage("robot/move", "left");});
    connect(movementControl, &MovementControl::moveright, [this]() { publishMessage("robot/move", "right");});
    connect(movementControl, &MovementControl::movereverse, [this]() { publishMessage("robot/move", "reverse");});
    connect(movementControl, &MovementControl::stopmovement, [this]() { publishMessage("robot/move", "stop");});
}

qt_mqtt_comm::~qt_mqtt_comm()
{
    delete m_client;
    delete movementControl;
}

void qt_mqtt_comm::initMqttClient(const QString &hostname, const quint16 port)
{
    m_client->setHostname(hostname);
    m_client->setPort(port);
    m_client->connectToHost();
}

void qt_mqtt_comm::publishMessage(const QString &topic, const QString &message)
{
    if(m_client->state() == QMqttClient::Connected)
    {
        m_client->publish(topic, message.toUtf8());
    }
    else
    {
        qWarning() << "Client not connected. cannot publish message";
    }
}

void qt_mqtt_comm::subscribeToTopic(const QString &topic)
{
    if(m_client->state() == QMqttClient::Connected)
    {
        if(!m_subscriptions.contains(topic))
        {
            QMqttSubscription *subscription = m_client->subscribe(topic);
            if(subscription)
            {
                m_subscriptions.insert(topic, subscription);
                // connect(subscription, &QMqttSubscription::messageReceived, this, &qt_mqtt_comm::onSubscribed);
                // connect(subscription, &QMqttSubscription::unsubscribed, this, &qt_mqtt_comm::onUnsubscribed);
                connect(subscription, &QMqttSubscription::stateChanged,
                        this, [topic](QMqttSubscription::SubscriptionState state) {
                            qDebug() << "Subscription state for" << topic << ":" << state;
                        });
                qDebug() << "Subscribed to topic:" << topic;
            }
            else
            {
                qWarning() << "Failed to subscribe to topic:" << topic;
            }
        }
        else
        {
            qWarning() << "Already subscribed to topic:" << topic;
        }
    }
    else
    {
        qWarning() << "Client not connected. cannot subscribe to topic";
    }
}

void qt_mqtt_comm::unsubscribeFromTopic(const QString &topic)
{
    if(m_subscriptions.contains(topic))
    {
        QMqttSubscription *subscription = m_subscriptions.value(topic);
        m_client->unsubscribe(subscription->topic());
        m_subscriptions.remove(topic);
        qDebug() << "Unsubscribed from topic:" << topic;
    }
    else
    {
        qWarning() << "Not subscribed to topic:" << topic;
    }
}

void qt_mqtt_comm::onConnected()
{
    qDebug() << "Connected to MQTT broker";
}
void qt_mqtt_comm::onDisconnected()
{
    qDebug() << "Disconnected from MQTT broker";
}
void qt_mqtt_comm::onMessageReceived(const QByteArray &message, const QMqttTopicName &topic)
{
    QString msg = QString::fromUtf8(message);
    QString top = topic.name();
    qDebug() << "Message received on topic" << top << ":" << msg;
    emit messageReceived(top, msg);
}
void qt_mqtt_comm::onSubscribed(const QString &topic)
{
    qDebug() << "Subscription confirmed for topic:" << topic;
}
void qt_mqtt_comm::onUnsubscribed(const QString &topic)
{
    qDebug() << "Unsubscription confirmed for topic:" << topic;
}
void qt_mqtt_comm::onErrorOccurred(QMqttClient::ClientError error)
{
    qWarning() << "MQTT Client error occurred:" << error;
}

void qt_mqtt_comm::connectToHost()
{
    qDebug() << "Connecting to MQTT broker at" << m_client->hostname() << ":" << m_client->port();
    m_client->connectToHost();
}

void qt_mqtt_comm::disconnectFromHost()
{
    qDebug() << "Disconnecting from MQTT broker";
    m_client->disconnectFromHost();
}


const QString qt_mqtt_comm::hostname() const
{
    return m_client->hostname();
}

void qt_mqtt_comm::setHostname(const QString &newHostname)
{
    m_client->setHostname(newHostname);
}

int qt_mqtt_comm::port() const
{
    return m_client->port();
}

void qt_mqtt_comm::setPort(const quint16 newPort)
{
    if (newPort < 0 || newPort > std::numeric_limits<quint16>::max()) {
        qWarning() << "Trying to set invalid port number";
        return;
    }
    m_client->setPort(static_cast<quint16>(newPort));
}

QMqttClient::ClientState qt_mqtt_comm::state() const
{
    return m_client->state();
}

void qt_mqtt_comm::setState(const QMqttClient::ClientState &newState)
{
    m_client->setState(newState);
}
