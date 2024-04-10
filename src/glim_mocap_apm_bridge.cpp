#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

//glim_rosのposeをardupilotに送信するためのブリッジ
class Bridge : public rclcpp::Node
{
public:
  Bridge() : Node("bridge") //, home_set_(false)
  {
    // QoS設定とパブリッシャー、サブスクリプションの作成
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone1/mavros/mocap/pose", qos);
    publisher_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone1/mavros/vision_pose/pose", qos);
    subscription_glim_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/glim_ros/pose", qos, std::bind(&Bridge::listener_callback, this, _1));
    subscription_mocap_pose = this->create_subscription<mocap_msgs::msg::RigidBodies>(
            "/rigid_bodies", qos, std::bind(&Bridge::listener_callback, this, _1));
    
    std::cout << "Bridge is running\n" << std::endl;
  }

private:
  void listener_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    publisher_->publish(*msg);
    publisher_2->publish(*msg);
  }

  rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr subscription_mocap_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_, publisher_2;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_glim_pose;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
  rclcpp::shutdown();
  return 0;
}
