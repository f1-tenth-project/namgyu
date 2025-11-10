#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <cmath>

class ScanCropNode : public rclcpp::Node {
public:
  ScanCropNode() : Node("scan_crop_node") {
    in_topic_   = declare_parameter<std::string>("in_topic",  "/scan0");
    out_topic_  = declare_parameter<std::string>("out_topic", "/scan_front");
    fov_deg_    = declare_parameter<double>("fov_deg", 50.0);    // ±25°
    center_deg_ = declare_parameter<double>("center_deg", 0.0);  // 전방 오프셋

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanCropNode::cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out_topic_, rclcpp::SensorDataQoS());
  }

private:
  void cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty() || msg->angle_increment == 0.0) return;

    const double fov = fov_deg_ * M_PI / 180.0;
    const double center = center_deg_ * M_PI / 180.0;

    const double a0 = center - 0.5 * fov;
    const double a1 = center + 0.5 * fov;

    auto idx = [&](double ang)->int {
      return (int)std::round((ang - msg->angle_min) / msg->angle_increment);
    };
    int i0 = std::clamp(idx(a0), 0, (int)msg->ranges.size()-1);
    int i1 = std::clamp(idx(a1), 0, (int)msg->ranges.size()-1);
    if (i1 < i0) std::swap(i0, i1);

    sensor_msgs::msg::LaserScan out;
    out.header = msg->header;
    out.angle_increment = msg->angle_increment;
    out.time_increment  = msg->time_increment;
    out.scan_time       = msg->scan_time;
    out.range_min       = msg->range_min;
    out.range_max       = msg->range_max;
    out.angle_min       = msg->angle_min + i0 * msg->angle_increment;
    out.angle_max       = msg->angle_min + i1 * msg->angle_increment;

    out.ranges.reserve(i1 - i0 + 1);
    for (int i = i0; i <= i1; ++i) out.ranges.push_back(msg->ranges[i]);

    if (!msg->intensities.empty()) {
      out.intensities.reserve(i1 - i0 + 1);
      for (int i = i0; i <= i1; ++i) out.intensities.push_back(msg->intensities[i]);
    }
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_;
  double fov_deg_, center_deg_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanCropNode>());
  rclcpp::shutdown();
  return 0;
}

