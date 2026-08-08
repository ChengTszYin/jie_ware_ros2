#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/empty.hpp"

class CostmapCleaner : public rclcpp::Node
{
public:
  CostmapCleaner() : Node("costmap_cleaner")
  {
    // Nav2 clear services
    global_client_ = this->create_client<std_srvs::srv::Empty>(
      "/global_costmap/clear_entirely_global_costmap");
    local_client_ = this->create_client<std_srvs::srv::Empty>(
      "/local_costmap/clear_entirely_local_costmap");

    sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&CostmapCleaner::initialPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Costmap cleaner ready (Nav2)");
  }

private:
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
  {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    if (global_client_->wait_for_service(std::chrono::seconds(1))) {
      global_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Cleared global costmap");
    }
    if (local_client_->wait_for_service(std::chrono::seconds(1))) {
      local_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Cleared local costmap");
    }
  }

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapCleaner>());
  rclcpp::shutdown();
  return 0;
}
