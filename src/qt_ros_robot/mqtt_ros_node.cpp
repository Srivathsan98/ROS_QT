#include "mqtt_ros_node.h"

MQTTNode::MQTTNode(const std::string &broker_address, const std::string &client_id)
    : Node("mqtt_ros_node"), broker_address_(broker_address), client_id_(client_id), client_(broker_address_, client_id_)
{
    mqtt::connect_options conn_opts_;
    try
    {
        client_.connect(conn_opts_)->wait();
        RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker at %s", broker_address_.c_str());

        // callback_ = [this](mqtt::const_message_ptr msg) { message_arrived(msg); };
        // client_.set_callback(callback_);

        client_.subscribe("robot/move", 1)->wait();
    }
    catch (const mqtt::exception &exc)
    {
        RCLCPP_ERROR(this->get_logger(), "Error connecting to MQTT broker: %s", exc.what());
    }
}

MQTTNode::~MQTTNode()
{
    try
    {
        client_.disconnect()->wait();
        RCLCPP_INFO(this->get_logger(), "Disconnected from MQTT broker");
    }
    catch (const mqtt::exception &exc)
    {
        RCLCPP_ERROR(this->get_logger(), "Error disconnecting from MQTT broker: %s", exc.what());
    }
}
void MQTTNode::publish(const std::string &topic, const std::string &payload)
{
    try
    {
        auto msg = mqtt::make_message(topic, payload);
        client_.publish(msg)->wait_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Published message to topic %s: %s", topic.c_str(), payload.c_str());
    }
    catch (const mqtt::exception &exc)
    {
        RCLCPP_ERROR(this->get_logger(), "Error publishing message: %s", exc.what());
    }
}
void MQTTNode::message_arrived(mqtt::const_message_ptr msg)
{
    RCLCPP_INFO(this->get_logger(), "Message arrived on topic %s: %s", msg->get_topic().c_str(), msg->to_string().c_str());
    // Process the message as needed
}
void MQTTNode::connect()
{
    try
    {
        client_.connect(conn_opts_)->wait();
        RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker at %s", broker_address_.c_str());
    }
    catch (const mqtt::exception &exc)
    {
        RCLCPP_ERROR(this->get_logger(), "Error connecting to MQTT broker: %s", exc.what());
    }
}
void MQTTNode::disconnect()
{
    try
    {
        client_.disconnect()->wait();
        RCLCPP_INFO(this->get_logger(), "Disconnected from MQTT broker");
    }
    catch (const mqtt::exception &exc)
    {
        RCLCPP_ERROR(this->get_logger(), "Error disconnecting from MQTT broker: %s", exc.what());
    }
}
