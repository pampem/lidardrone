#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_long.hpp"

class DroneController : public rclcpp::Node {
public:
  DroneController() : Node("drone_controller") {
    joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&DroneController::joy_callback, this, std::placeholders::_1));

    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("drone1/mavros/set_mode");
    arming_client = this->create_client<mavros_msgs::srv::CommandBool>("drone1/mavros/cmd/arming");
    velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>("drone1/mavros/cmd/takeoff");
    land_client = this->create_client<mavros_msgs::srv::CommandTOL>("drone1/mavros/cmd/land");
    sethome_cmd_client = this->create_client<mavros_msgs::srv::CommandLong>("drone1/mavros/cmd/command");
 
    arm_and_takeoff();
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::Twist velocity_msg;
    // X botton = land
    if (msg->buttons[0] == 1) {
      land_drone();
    }
    // A botton = takeoff
    if (msg->buttons[1] == 1) {
      arm_and_takeoff();
    }

    float left_stick_lr = msg->axes[0]; // Left stick left/right
    float left_stick_ud = msg->axes[1]; // Left stick up/down
    float right_stick_lr = msg->axes[2]; // Light stick left/right
    float right_stick_ud = msg->axes[3]; // Light stick up/down
  
  
    velocity_msg.linear.x = left_stick_ud; // 前後の移動
    velocity_msg.linear.y = left_stick_lr; // 左右の移動

    velocity_msg.linear.z = right_stick_ud; // 高度移動
    velocity_msg.angular.z = right_stick_lr; // 回転

    velocity_publisher->publish(velocity_msg);
  }

  void arm_and_takeoff(){
    // サービスが利用可能になるまで待機
    while (!arming_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for arming service to become available");
    }
    while (!set_mode_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for set_mode service to become available");
    }

    // GUIDEDへの切り替えを試みる
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "GUIDED";
    auto future = set_mode_client->async_send_request(request);
    
    // Arm Drone
    auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arming_request->value = true;
    auto arming_future = arming_client->async_send_request(arming_request);

    rclcpp::sleep_for(std::chrono::seconds(3)); 
    
    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    takeoff_request->altitude = 3.3; //takeoff altitude
    takeoff_request->latitude = 0; // 現在位置を使用
    takeoff_request->longitude = 0; // 現在位置を使用
    takeoff_request->min_pitch = 0;
    takeoff_request->yaw = 0;
    auto takeoff_future = takeoff_client->async_send_request(takeoff_request);

    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Drone armed and takeoff");
  }

  void land_drone() {
    // サービスが利用可能になるまで待機
    while (!land_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for land service to become available");
    }

    // 着陸コマンドを準備
    auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_request->altitude = 0; // 着陸時の目標高度
    land_request->latitude = 0; // 0を指定することで現在位置を使用
    land_request->longitude = 0; // 0を指定することで現在位置を使用
    land_request->min_pitch = 0; // 最小ピッチ角、通常は0でOK
    land_request->yaw = 0; // 着陸時のヨー角、通常は現在のヨー角を維持
    
    auto land_future = land_client->async_send_request(land_request);

    RCLCPP_INFO(this->get_logger(), "Drone landing initiated");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr sethome_cmd_client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
