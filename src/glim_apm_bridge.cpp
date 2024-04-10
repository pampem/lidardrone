#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_long.hpp"

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
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone1/mavros/mocap/pose", qos);
        publisher_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone1/mavros/vision_pose/pose", qos);
        subscription_glim_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/glim_ros/pose", qos, std::bind(&Bridge::listener_callback, this, _1));
        // cmd_client_ = this->create_client<mavros_msgs::srv::CommandLong>("drone1/mavros/cmd/command");

        // // タイマーの作成。ここでは、例として30秒後にホームポジションを設定する
        // timer_ = this->create_wall_timer(30s, std::bind(&Bridge::set_home_position_from_timer, this));
        std::cout << "Bridge is running\n" << std::endl;
    }

private:
    void listener_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        publisher_->publish(*msg);
        publisher_2->publish(*msg);
    }

    // void set_home_position_from_timer()
    // {
    //     if (!home_set_)
    //     {
    //         // 仮のPoseStampedオブジェクトを使用してset_home_positionを呼び出す
    //         // 実際には、このPoseStampedはドローンの現在位置を正確に反映している必要があります
    //         auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    //         // msg->pose.position.x, msg->pose.position.y, msg->pose.position.zを設定
    //         set_home_position(msg);
    //         home_set_ = true;
    //         // タイマーをキャンセル
    //         timer_->cancel();
    //     }
    // }

    // void set_home_position(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    // {
    //     if (!cmd_client_->wait_for_service(1s))
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Command service not available.");
    //         return;
    //     }
    //     auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    //     request->command = 179; // MAV_CMD_DO_SET_HOME
    //     request->param1 = 1; // Use current position
    //     // 他のパラメータは必要に応じて設定

    //     auto result = cmd_client_->async_send_request(request);
    //     // ここでの応答は非同期ですが、必要に応じて応答をチェックすることができます
    // }

    // bool home_set_;
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
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
