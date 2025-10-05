#include <mqtt/async_client.h>
#include <thread>
#include <random>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
class MQTTNode : public rclcpp::Node
{
public:
    MQTTNode(const std::string &broker_address, const std::string &client_id);
    ~MQTTNode();
    void publish(const std::string &topic, const std::string &payload);

private:
    std::string broker_address_;
    std::string client_id_;
    mqtt::async_client client_;
    mqtt::connect_options conn_opts_;
    void connect();
    void disconnect();
    void message_arrived(mqtt::const_message_ptr msg);
    std::thread mqtt_thread_;
    bool running_;
    const std::string SERVER_ADDRESS{"tcp://localhost:1883"}; // set to PC IP for communication between PI and PC
    const std::string CLIENT_ID{"ros2_node"};
};