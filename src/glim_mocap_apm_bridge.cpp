#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Bridge : public rclcpp::Node
{
public:
  Bridge() : Node("bridge"), last_glim_pose_time_(this->now())
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone1/mavros/mocap/pose", qos);
    publisher_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone1/mavros/vision_pose/pose", qos);
    subscription_glim_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/glim_ros/pose", qos, std::bind(&Bridge::glim_pose_callback, this, _1));
    subscription_mocap_pose = this->create_subscription<mocap_msgs::msg::RigidBodies>(
            "/rigid_bodies", qos, std::bind(&Bridge::mocap_pose_callback, this, _1));

    timer_ = this->create_wall_timer(500ms, std::bind(&Bridge::timer_callback, this));
    
    std::cout << "Bridge is running\n" << std::endl;
  }

private:
  void glim_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_glim_pose_time_ = this->now();
    publisher_->publish(*msg);
    publisher_2->publish(*msg);
  }

  void mocap_pose_callback(const mocap_msgs::msg::RigidBodies::SharedPtr msg)
  {
    if (!msg->rigidbodies.empty()) {
      auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_stamped->header.stamp = msg->header.stamp;  // コピーするタイムスタンプ
      pose_stamped->header.frame_id = "map";  // 適切なフレームIDを設定
      pose_stamped->pose = msg->rigidbodies[0].pose;  // 最初のRigidBodyのポーズをコピー

      // publisher_->publish(*pose_stamped);
      // publisher_2->publish(*pose_stamped);
      last_mocap_pose_ = pose_stamped;
    }
  }

  void timer_callback()
  {
    if (this->now() - last_glim_pose_time_ > 0.1s) {
      // If last Glim pose was received more than 0.5 seconds ago, publish last mocap pose
      if (last_mocap_pose_) {
        publisher_->publish(*last_mocap_pose_);
        publisher_2->publish(*last_mocap_pose_);
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_glim_pose;
  rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr subscription_mocap_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_2;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_glim_pose_time_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> last_mocap_pose_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bridge>());
  rclcpp::shutdown();
  return 0;
}
