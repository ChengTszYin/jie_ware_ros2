#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <cmath>

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter() : Node("lidar_filter_node")
  {
    this->declare_parameter<std::string>("source_topic", "/scan");
    this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
    this->declare_parameter<double>("outlier_threshold", 0.1);

    source_topic_ = this->get_parameter("source_topic").as_string();
    pub_topic_ = this->get_parameter("pub_topic").as_string();
    outlier_threshold_ = this->get_parameter("outlier_threshold").as_double();

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic_, 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      source_topic_, 10,
      std::bind(&LidarFilter::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Lidar filter started: %s -> %s",
                source_topic_.c_str(), pub_topic_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    int n = scan->ranges.size();
    if (n < 3) {
      pub_->publish(*scan);
      return;
    }

    auto new_scan = *scan;   // copy

    for (int i = 1; i < n - 1; ++i) {
      float prev = new_scan.ranges[i-1];
      float curr = new_scan.ranges[i];
      float next = new_scan.ranges[i+1];

      bool valid = std::isfinite(curr) &&
                   curr >= new_scan.range_min &&
                   curr <= new_scan.range_max;

      if (!valid) continue;

      if (std::abs(curr - prev) > outlier_threshold_ &&
          std::abs(curr - next) > outlier_threshold_)
      {
        new_scan.ranges[i] = std::numeric_limits<float>::infinity();
        if (!new_scan.intensities.empty() && i < static_cast<int>(new_scan.intensities.size())) {
          new_scan.intensities[i] = 0.0f;
        }
      }
    }
    pub_->publish(new_scan);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::string source_topic_, pub_topic_;
  double outlier_threshold_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
  return 0;
}
