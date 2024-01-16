#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OccupancyGridMapper : public rclcpp::Node
{
public:
    OccupancyGridMapper() : Node("occupancy_grid_mapper")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/glim_ros/map", 10, std::bind(&OccupancyGridMapper::listener_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy", 10);
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // ここで点群データを処理してOccupancy Gridを生成
        // ...

        // Occupancy Gridをパブリッシュ
        auto grid_msg = nav_msgs::msg::OccupancyGrid();
        grid_msg.header = msg->header;
        // grid_msg.info.resolution, grid_msg.info.width, grid_msg.info.height, grid_msg.data を設定
        // ...
        publisher_->publish(grid_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridMapper>());
    rclcpp::shutdown();
    return 0;
}
