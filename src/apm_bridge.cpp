#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Bridge : public rclcpp::Node
{
public:
    Bridge() : Node("bridge")
    {
        // QoS設定を作成
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // QoSを使用してパブリッシャーを作成
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/mocap/pose", qos);
        publisher_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/vision_pose/pose", qos);

        // QoSを使用してサブスクリプションを作成
        subscription_ = this->create_subscription<mocap_msgs::msg::RigidBodies>(
            "/rigid_bodies", qos, std::bind(&Bridge::listener_callback, this, _1));
    }

private:
    void listener_callback(const mocap_msgs::msg::RigidBodies::SharedPtr msg)
    {
        if (!msg->rigidbodies.empty()) {
            auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
            pose_stamped->header.stamp = msg->header.stamp;  // コピーするタイムスタンプ
            pose_stamped->header.frame_id = "map";  // 適切なフレームIDを設定
            pose_stamped->pose = msg->rigidbodies[0].pose;  // 最初のRigidBodyのポーズをコピー

            publisher_->publish(*pose_stamped);
            publisher_2->publish(*pose_stamped);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_2;
    rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bridge>());
    rclcpp::shutdown();
    return 0;
}
