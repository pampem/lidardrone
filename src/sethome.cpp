#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include "mavros_msgs/srv/command_long.hpp"
class SetHomeClient : public rclcpp::Node, public std::enable_shared_from_this<SetHomeClient>
{
public:
    SetHomeClient() : Node("set_home_client")
    {
        sethome_cmd_client = this->create_client<mavros_msgs::srv::CommandLong>("drone1/mavros/cmd/command");
        set_home_position();
    }

private:
    void set_home_position()
    {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->command = 179; // MAV_CMD_DO_SET_HOME
    request->param1 = 1; // Use current position
    // 他のパラメータは必要に応じて設定

    auto result = sethome_cmd_client->async_send_request(request);
    // ここでの応答は非同期ですが、必要に応じて応答をチェックすることができます
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Drone home position set");
    }

    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr sethome_cmd_client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetHomeClient>());
    rclcpp::shutdown();
    return 0;
}
