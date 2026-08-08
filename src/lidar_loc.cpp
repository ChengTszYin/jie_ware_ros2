#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>

class LidarLoc : public rclcpp::Node
{
public:
  LidarLoc()
  : Node("lidar_loc"),
    lidar_x_(250.0f),
    lidar_y_(250.0f),
    lidar_yaw_(0.0f),
    clear_countdown_(-1),
    scan_count_(0)
  {
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("laser_frame", "laser");
    this->declare_parameter<std::string>("laser_topic", "scan");

    base_frame_  = this->get_parameter("base_frame").as_string();
    odom_frame_  = this->get_parameter("odom_frame").as_string();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    laser_topic_ = this->get_parameter("laser_topic").as_string();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&LidarLoc::mapCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_, 10,
      std::bind(&LidarLoc::scanCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&LidarLoc::initialPoseCallback, this, std::placeholders::_1));

    clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>(
      "/global_costmap/clear_entirely_global_costmap");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&LidarLoc::poseTf, this));

    RCLCPP_INFO(this->get_logger(), "lidar_loc started");
  }

private:
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (map_msg_.info.resolution <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Map information is invalid or not received yet");
      return;
    }

    double map_x = msg->pose.pose.position.x;
    double map_y = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    lidar_x_ = static_cast<float>((map_x - map_msg_.info.origin.position.x) /
                                  map_msg_.info.resolution - map_roi_info_.x_offset);
    lidar_y_ = static_cast<float>((map_y - map_msg_.info.origin.position.y) /
                                  map_msg_.info.resolution - map_roi_info_.y_offset);
    lidar_yaw_ = static_cast<float>(-yaw);

    clear_countdown_ = 30;
    RCLCPP_INFO(this->get_logger(), "Initial pose received: (%.1f, %.1f) yaw=%.2f",
                lidar_x_, lidar_y_, lidar_yaw_);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = *msg;
    cropMap();
    processMap();
    RCLCPP_INFO(this->get_logger(), "Map received and processed");
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_points_.clear();
    double angle = msg->angle_min;

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        base_frame_, laser_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", ex.what());
      return;
    }

    // Detect inverted lidar
    tf2::Quaternion q_lidar;
    tf2::fromMsg(transform_stamped.transform.rotation, q_lidar);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);

    const double tolerance = 0.1;
    bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
    lidar_is_inverted = lidar_is_inverted &&
                        !(std::abs(std::abs(pitch) - M_PI) < tolerance);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
        float x_laser = msg->ranges[i] * std::cos(angle);
        float y_laser = -msg->ranges[i] * std::sin(angle);

        geometry_msgs::msg::PointStamped point_laser;
        point_laser.header.frame_id = laser_frame_;
        point_laser.header.stamp = msg->header.stamp;
        point_laser.point.x = x_laser;
        point_laser.point.y = y_laser;
        point_laser.point.z = 0.0;

        geometry_msgs::msg::PointStamped point_base;
        tf2::doTransform(point_laser, point_base, transform_stamped);

        float x = point_base.point.x / map_msg_.info.resolution;
        float y = point_base.point.y / map_msg_.info.resolution;

        if (lidar_is_inverted) {
          x = -x;
          y = -y;
        }

        scan_points_.emplace_back(x, y);
      }
      angle += msg->angle_increment;
    }

    if (scan_count_ == 0) {
      ++scan_count_;
    }

    // Matching loop
    while (rclcpp::ok()) {
      if (map_cropped_.empty() || map_temp_.empty()) {
        break;
      }

      std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;
      int max_sum = 0;
      float best_dx = 0.0f, best_dy = 0.0f, best_dyaw = 0.0f;

      for (const auto & point : scan_points_) {
        float rx = point.x * std::cos(lidar_yaw_) - point.y * std::sin(lidar_yaw_);
        float ry = point.x * std::sin(lidar_yaw_) + point.y * std::cos(lidar_yaw_);
        transform_points.emplace_back(rx + lidar_x_, lidar_y_ - ry);

        float yaw_cw = lidar_yaw_ + deg_to_rad_;
        rx = point.x * std::cos(yaw_cw) - point.y * std::sin(yaw_cw);
        ry = point.x * std::sin(yaw_cw) + point.y * std::cos(yaw_cw);
        clockwise_points.emplace_back(rx + lidar_x_, lidar_y_ - ry);

        float yaw_ccw = lidar_yaw_ - deg_to_rad_;
        rx = point.x * std::cos(yaw_ccw) - point.y * std::sin(yaw_ccw);
        ry = point.x * std::sin(yaw_ccw) + point.y * std::cos(yaw_ccw);
        counter_points.emplace_back(rx + lidar_x_, lidar_y_ - ry);
      }

      std::vector<cv::Point2f> offsets = {{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      std::vector<std::vector<cv::Point2f>> point_sets = {
        transform_points, clockwise_points, counter_points};
      std::vector<float> yaw_offsets = {0.0f, deg_to_rad_, -deg_to_rad_};

      for (size_t i = 0; i < offsets.size(); ++i) {
        for (size_t j = 0; j < point_sets.size(); ++j) {
          int sum = 0;
          for (const auto & p : point_sets[j]) {
            float px = p.x + offsets[i].x;
            float py = p.y + offsets[i].y;
            if (px >= 0 && px < map_temp_.cols && py >= 0 && py < map_temp_.rows) {
              sum += map_temp_.at<uchar>(static_cast<int>(py), static_cast<int>(px));
            }
          }
          if (sum > max_sum) {
            max_sum = sum;
            best_dx = offsets[i].x;
            best_dy = offsets[i].y;
            best_dyaw = yaw_offsets[j];
          }
        }
      }

      lidar_x_ += best_dx;
      lidar_y_ += best_dy;
      lidar_yaw_ += best_dyaw;

      if (check(lidar_x_, lidar_y_, lidar_yaw_)) {
        break;
      }
    }

    if (clear_countdown_ > -1) {
      --clear_countdown_;
    }
    if (clear_countdown_ == 0) {
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      if (clear_costmaps_client_->service_is_ready()) {
        clear_costmaps_client_->async_send_request(request);
      }
    }
  }

  void cropMap()
  {
    auto info = map_msg_.info;
    int x_max = info.width / 2, x_min = info.width / 2;
    int y_max = info.height / 2, y_min = info.height / 2;
    bool first = true;

    cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128));

    for (int y = 0; y < static_cast<int>(info.height); ++y) {
      for (int x = 0; x < static_cast<int>(info.width); ++x) {
        int index = y * info.width + x;
        map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg_.data[index]);

        if (map_msg_.data[index] == 100) {
          if (first) {
            x_max = x_min = x;
            y_max = y_min = y;
            first = false;
          } else {
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
          }
        }
      }
    }

    int cen_x = (x_min + x_max) / 2;
    int cen_y = (y_min + y_max) / 2;
    int half_w = std::abs(x_max - x_min) / 2 + 50;
    int half_h = std::abs(y_max - y_min) / 2 + 50;

    int origin_x = std::max(0, cen_x - half_w);
    int origin_y = std::max(0, cen_y - half_h);
    int width  = std::min(static_cast<int>(info.width) - origin_x, half_w * 2);
    int height = std::min(static_cast<int>(info.height) - origin_y, half_h * 2);

    cv::Rect roi(origin_x, origin_y, width, height);
    map_cropped_ = map_raw(roi).clone();

    map_roi_info_.x_offset = origin_x;
    map_roi_info_.y_offset = origin_y;
    map_roi_info_.width = width;
    map_roi_info_.height = height;

    geometry_msgs::msg::PoseWithCovarianceStamped init;
    init.pose.pose.orientation.w = 1.0;
    initialPoseCallback(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(init));
  }

  cv::Mat createGradientMask(int size)
  {
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; ++y) {
      for (int x = 0; x < size; ++x) {
        double distance = std::hypot(x - center, y - center);
        int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
        mask.at<uchar>(y, x) = value;
      }
    }
    return mask;
  }

  void processMap()
  {
    if (map_cropped_.empty()) return;

    map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
    cv::Mat gradient_mask = createGradientMask(101);

    for (int y = 0; y < map_cropped_.rows; ++y) {
      for (int x = 0; x < map_cropped_.cols; ++x) {
        if (map_cropped_.at<uchar>(y, x) == 100) {
          int left   = std::max(0, x - 50);
          int top    = std::max(0, y - 50);
          int right  = std::min(map_cropped_.cols - 1, x + 50);
          int bottom = std::min(map_cropped_.rows - 1, y + 50);

          cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
          cv::Mat region = map_temp_(roi);

          int mask_left = 50 - (x - left);
          int mask_top  = 50 - (y - top);
          cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
          cv::Mat mask = gradient_mask(mask_roi);

          cv::max(region, mask, region);
        }
      }
    }
  }

  bool check(float x, float y, float yaw)
  {
    if (x == 0.0f && y == 0.0f && yaw == 0.0f) {
      data_queue_.clear();
      return true;
    }

    data_queue_.emplace_back(x, y, yaw);
    if (data_queue_.size() > max_size_) {
      data_queue_.pop_front();
    }

    if (data_queue_.size() == max_size_) {
      auto & first = data_queue_.front();
      auto & last  = data_queue_.back();

      float dx   = std::abs(std::get<0>(last) - std::get<0>(first));
      float dy   = std::abs(std::get<1>(last) - std::get<1>(first));
      float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

      if (dx < 5.0f && dy < 5.0f && dyaw < 5.0f * deg_to_rad_) {
        data_queue_.clear();
        return true;
      }
    }
    return false;
  }

  void poseTf()
  {
    if (scan_count_ == 0 || map_cropped_.empty() ||
        map_msg_.data.empty() || map_msg_.info.resolution <= 0.0) {
      return;
    }

    double map_x = (lidar_x_ + map_roi_info_.x_offset) * map_msg_.info.resolution +
                   map_msg_.info.origin.position.x;
    double map_y = (lidar_y_ + map_roi_info_.y_offset) * map_msg_.info.resolution +
                   map_msg_.info.origin.position.y;
    double map_yaw = -lidar_yaw_;

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(map_x, map_y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_yaw);
    map_to_base.setRotation(q);

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    try {
      odom_to_base_msg = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Cannot get transform %s -> %s: %s",
                           odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    tf2::Transform odom_to_base;
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base);

    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = this->now();
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_->sendTransform(map_to_odom_msg);
  }

  // Members
  std::string base_frame_, odom_frame_, laser_frame_, laser_topic_;

  nav_msgs::msg::OccupancyGrid map_msg_;
  struct MapRoi {
    int x_offset = 0;
    int y_offset = 0;
    int width = 0;
    int height = 0;
  } map_roi_info_;

  cv::Mat map_cropped_, map_temp_;
  std::vector<cv::Point2f> scan_points_;

  float lidar_x_, lidar_y_, lidar_yaw_;
  const float deg_to_rad_ = static_cast<float>(M_PI / 180.0);
  int clear_countdown_;
  int scan_count_;

  std::deque<std::tuple<float, float, float>> data_queue_;
  const size_t max_size_ = 10;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLoc>());
  rclcpp::shutdown();
  return 0;
}