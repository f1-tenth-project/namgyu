// pure_pursuit_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/laser_scan.hpp> // B 스캔용
#include <algorithm>
#include <cmath>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    // Params (기존)
    lookahead_ = declare_parameter<double>("lookahead", 1.5);     // [m]
    wheelbase_ = declare_parameter<double>("wheelbase", 0.34);    // [m]
    v_min_     = declare_parameter<double>("speed_min", 1.0);     // [m/s]
    v_max_     = declare_parameter<double>("speed_max", 25.0);     // [m/s] // 4.0
    k_speed_   = declare_parameter<double>("k_speed",  2.5);      // curvature → speed (미사용 가능)
    k_accel_   = declare_parameter<double>("k_accel",  2.0);      // P gain for (v_ref - v) //2.5
    a_min_     = declare_parameter<double>("accel_min",-3.0);     // [m/s^2]
    a_max_     = declare_parameter<double>("accel_max", 3.0);     // [m/s^2]
    center_path_topic_ = declare_parameter<std::string>("center_path_topic", "center_path");
    left_path_topic_   = declare_parameter<std::string>("left_boundary", "left_boundary");
    right_path_topic_  = declare_parameter<std::string>("right_boundary","right_boundary");
    odom_topic_        = declare_parameter<std::string>("odom_topic", "odom0");
    drive_topic_       = declare_parameter<std::string>("drive_topic", "ackermann_cmd0");
    a_ref_up_   = declare_parameter<double>("a_ref_up",   6.0);  // 가속 방향 v_ref 상승 한계 [m/s^2]
    a_ref_down_ = declare_parameter<double>("a_ref_down", 4.0);  // 감속 방향 v_ref 하향 한계 [m/s^2]
    v_deadband_ = declare_parameter<double>("v_deadband", 0.15); // 속도 오차 데드밴드 [m/s]


    // ★ 새 파라미터: 동적 Ld, 곡률속도, TTC(B 스캔 사용)
    ay_max_        = declare_parameter<double>("ay_max", 6.0);           // 횡가속 한계
    ld0_           = declare_parameter<double>("ld0", 1.2);
    kv_ld_         = declare_parameter<double>("kv_ld", 0.25);
    ld_min_        = declare_parameter<double>("ld_min", 0.8);
    ld_max_        = declare_parameter<double>("ld_max", 2.2);
    ttc_safe_      = declare_parameter<double>("ttc_safe", 1.2);         // 안전 TTC
    k_ttc_         = declare_parameter<double>("k_ttc", 3.0);            // TTC→속도 스케일
    ttc_scan_topic_= declare_parameter<std::string>("ttc_scan_topic", "scan_front"); // B 토픽
    tau_v_         = declare_parameter<double>("tau_v", 0.15);           // 속도 LPF 시간상수[s]
    ttc_gate_      = declare_parameter<double>("ttc_gate", 1.5); //추가됨

    // Pubs/Subs
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    sub_center_ = create_subscription<nav_msgs::msg::Path>(
        center_path_topic_, rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::Path::SharedPtr msg){ center_path_ = *msg; });

    sub_left_ = create_subscription<nav_msgs::msg::Path>(
        left_path_topic_, rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::Path::SharedPtr msg){ left_path_ = *msg; });

    sub_right_ = create_subscription<nav_msgs::msg::Path>(
        right_path_topic_, rclcpp::QoS(1).reliable(),
        [this](nav_msgs::msg::Path::SharedPtr msg){ right_path_ = *msg; });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg){
          odom_ = *msg; has_odom_ = true;
        });

    // ★ B 스캔 구독(/scan_front). 전방만 잘린 스캔이 들어온다고 가정
    sub_ttc_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        ttc_scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
          last_ttc_scan_ = *msg; has_ttc_scan_ = true;
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&PurePursuitNode::onTimer, this));
  }

private:
  // ★ 보조 함수: 최소 TTC
  static double computeMinTTC(const sensor_msgs::msg::LaserScan& s, double v){
    if (v < 0.05) return 1e9;
    double ang = s.angle_min, min_ttc = 1e9;
    for (size_t i=0;i<s.ranges.size(); ++i, ang += s.angle_increment){
      const float r = s.ranges[i];
      if (!std::isfinite(r)) continue;
      const double v_rel = v * std::cos(ang);
      if (v_rel <= 0) continue;
      const double ttc = r / std::max(1e-3, v_rel);
      if (ttc < min_ttc) min_ttc = ttc;
    }
    return min_ttc;
  }
  // ★ 보조 함수: 1차 LPF
  static double lpf(double x, double x_prev, double tau, double dt){
    const double a = dt/(tau+dt);
    return x_prev + a*(x - x_prev);
  }

  void onTimer() {
    if (!has_odom_ || center_path_.poses.empty()) return;

    // Current pose & yaw
    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    const double x = p.x, y = p.y;
    const double v = odom_.twist.twist.linear.x;

    // ★ 동적 룩어헤드 (속도 기반)
    const double Ld_target = std::clamp(ld0_ + kv_ld_ * v, ld_min_, ld_max_);

    // Find lookahead target on center path
    int target_idx = findLookaheadIndex(center_path_, x, y, Ld_target); // ★ lookahead_ → Ld_target
    if (target_idx < 0) return;

    const auto &tp = center_path_.poses[target_idx].pose.position;

    // Target in vehicle frame (x forward, y left)
    const double dx = tp.x - x;
    const double dy = tp.y - y;
    const double xL =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (xL <= 0.01) return;

    // Pure Pursuit geometry
    const double Ld = std::hypot(xL, yL);
    const double curvature = 2.0 * yL / (Ld * Ld);
    const double steer = std::atan(wheelbase_ * curvature);

    // ★ (1) 곡률 기반 속도 상한 (ay <= ay_max)
    double v_curv = (std::abs(curvature) < 1e-4) ? v_max_
                   : std::sqrt(std::max(1e-6, ay_max_ / std::abs(curvature)));
    v_curv = std::clamp(v_curv, v_min_, v_max_);
    
    // ★ (2) TTC 기반 속도 상한 (게이트 적용)
    double v_ttc = v_max_;
    if (has_ttc_scan_) {
    const double ttc_min = computeMinTTC(last_ttc_scan_, std::max(0.0, v));
    	if (ttc_min < ttc_gate_) { // 위험 구간에서만 TTC 제한 적용
    	v_ttc = std::clamp(k_ttc_ * (ttc_min - ttc_safe_), 0.0, v_max_);
    	} else {
    	v_ttc = v_max_; // 위협 없으면 TTC 비활성
  	}
    }
    // ★ (3) 최종 속도 = min(곡률, TTC) → LPF
    const double v_ref_raw = std::min(v_curv, v_ttc);
    const double v_ref = lpf(v_ref_raw, v_cmd_prev_, tau_v_, 0.01); // dt=0.01s(10ms 타이머)
    v_cmd_prev_ = v_ref;

    // Acceleration command (P-control), clamped
    double a_cmd = k_accel_ * (v_ref - v);
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;

    // Publish acceleration-based command
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.acceleration   = a_cmd;
    cmd.drive.speed = 0.0; // 시뮬이 speed 제어면 이 줄 사용
    
    drive_pub_->publish(cmd);
  }

  // Return first index whose distance >= Ld from (x,y); -1 if none
  static int findLookaheadIndex(const nav_msgs::msg::Path &path,
                                double x, double y, double Ld) {
    int closest = 0;
    double best = 1e18;
    for (size_t i=0; i<path.poses.size(); ++i) {
      const auto &pt = path.poses[i].pose.position;
      double d2 = (pt.x - x)*(pt.x - x) + (pt.y - y)*(pt.y - y);
      if (d2 < best) { best = d2; closest = static_cast<int>(i); }
    }
    double accum = 0.0;
    for (size_t i=closest; i+1<path.poses.size(); ++i) {
      const auto &a = path.poses[i].pose.position;
      const auto &b = path.poses[i+1].pose.position;
      accum += std::hypot(b.x - a.x, b.y - a.y);
      if (accum >= Ld) return static_cast<int>(i+1);
    }
    if (!path.poses.empty()) return static_cast<int>(path.poses.size()-1);
    return -1;
  }

  // Members
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_center_, sub_left_, sub_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ttc_scan_; // ★
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path center_path_, left_path_, right_path_;
  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::LaserScan last_ttc_scan_; // ★
  bool has_odom_{false}, has_ttc_scan_{false}; // ★

  // Params
  double lookahead_, wheelbase_;
  double v_min_, v_max_, k_speed_;
  double k_accel_, a_min_, a_max_;
  std::string center_path_topic_, left_path_topic_, right_path_topic_, odom_topic_, drive_topic_;
  // ★ 새 파라미터 & 상태
  double ay_max_, ld0_, kv_ld_, ld_min_, ld_max_, ttc_safe_, k_ttc_, tau_v_, ttc_gate_;
  std::string ttc_scan_topic_;
  double v_cmd_prev_{0.0};
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}

