#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>

class PoseAccuracyEvaluator : public rclcpp::Node
{
public:
  PoseAccuracyEvaluator() : Node("pose_accuracy_evaluator"), outfile_("evaluation_results.csv")
  {
  // `BEST_EFFORT`の信頼性でQoS設定を作成
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    gt_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone1/mavros/local_position/pose", qos,
        std::bind(&PoseAccuracyEvaluator::gtPoseCallback, this, std::placeholders::_1));

    // Pose to be Evaluated Subscription
    eval_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/glim_ros/pose", 10,
        std::bind(&PoseAccuracyEvaluator::evalPoseCallback, this, std::placeholders::_1));

    pose_array_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/iris_pose", 10,
        std::bind(&PoseAccuracyEvaluator::evalPoseArrayCallback, this, std::placeholders::_1));

    outfile_ << "Timestamp,PositionDifference,OrientationDifference\n";
  }

  ~PoseAccuracyEvaluator()
  {
    outfile_.close();
  }

private:
  void gtPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_gt_pose_ = msg;
  }

  void evalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!last_gt_pose_) {
      RCLCPP_WARN(this->get_logger(), "GT pose not received yet.");
      return;
    }

    // 評価結果を計算
    double position_diff = calculatePositionDifference(last_gt_pose_->pose, msg->pose);
    double orientation_diff = calculateOrientationDifference(last_gt_pose_->pose, msg->pose);

    // 現在の時刻を取得
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // 評価結果をファイルに書き込む
    outfile_ << milliseconds << "," << position_diff << "," << orientation_diff << "\n";

    RCLCPP_INFO(this->get_logger(),
                "Position Difference: %f, Orientation Difference: %f",
                position_diff, orientation_diff);
  }

  double calculatePositionDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
  {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
  }

  double calculateOrientationDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
  {
    tf2::Quaternion quat1, quat2;
    tf2::fromMsg(pose1.orientation, quat1);
    tf2::fromMsg(pose2.orientation, quat2);
    double angle_diff = quat1.angle(quat2);
    return angle_diff;
  }

  void evalPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!last_gt_pose_) {
      RCLCPP_WARN(this->get_logger(), "GT pose not received yet.");
      return;
    }

    for (auto pose : msg->poses) {
      double position_diff = calculatePositionDifference(last_gt_pose_->pose, pose);
      double orientation_diff = calculateOrientationDifference(last_gt_pose_->pose, pose);

      // 現在の時刻を取得
      auto now = std::chrono::system_clock::now();
      auto duration = now.time_since_epoch();
      auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

      // 評価結果をファイルに書き込む
      outfile_ << milliseconds << "," << position_diff << "," << orientation_diff << "\n";

      RCLCPP_INFO(this->get_logger(),
                  "Position Difference: %f, Orientation Difference: %f",
                  position_diff, orientation_diff);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eval_pose_subscription_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_gt_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_subscription;
  std::ofstream outfile_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseAccuracyEvaluator>());
  rclcpp::shutdown();
  return 0;
}
