// pure_pursuit_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    // ----- 기본 PP / 토픽 파라미터 -----
    lookahead_ = declare_parameter<double>("lookahead", 1.5);           // (동적 Ld가 대신 사용됨)
    wheelbase_ = declare_parameter<double>("wheelbase", 0.34);
    v_min_     = declare_parameter<double>("speed_min", 1.0);
    v_max_     = declare_parameter<double>("speed_max", 7.0);           // 안전 기본값(직선 하드캡)
    k_speed_   = declare_parameter<double>("k_speed",  2.5);            // (미사용 가능)
    k_accel_   = declare_parameter<double>("k_accel",  4.0);            // P이득
    a_min_     = declare_parameter<double>("accel_min",-5.0);
    a_max_     = declare_parameter<double>("accel_max", 8.0);
    center_path_topic_ = declare_parameter<std::string>("center_path_topic", "center_path");
    left_path_topic_   = declare_parameter<std::string>("left_boundary",  "left_boundary");
    right_path_topic_  = declare_parameter<std::string>("right_boundary", "right_boundary");
    odom_topic_        = declare_parameter<std::string>("odom_topic",     "odom0");
    drive_topic_       = declare_parameter<std::string>("drive_topic",    "ackermann_cmd0");

    // ----- 고도화 파라미터: Ld/곡률속도/TTC -----
    ay_max_        = declare_parameter<double>("ay_max",   6.0);         // 횡가속 제한
    ld0_           = declare_parameter<double>("ld0",      1.2);
    kv_ld_         = declare_parameter<double>("kv_ld",    0.22);
    ld_min_        = declare_parameter<double>("ld_min",   0.8);
    ld_max_        = declare_parameter<double>("ld_max",   2.0);

    ttc_safe_      = declare_parameter<double>("ttc_safe", 0.9);
    k_ttc_         = declare_parameter<double>("k_ttc",    8.0);
    ttc_gate_      = declare_parameter<double>("ttc_gate", 1.1);
    ttc_scan_topic_= declare_parameter<std::string>("ttc_scan_topic", "scan_front");

    tau_v_         = declare_parameter<double>("tau_v",    0.10);        // v_ref LPF

    // ----- v_ref 안정화(래이트 제한/데드밴드) -----
    a_ref_up_      = declare_parameter<double>("a_ref_up",   6.0);       // v_ref 상승 한계 [m/s^2]
    a_ref_down_    = declare_parameter<double>("a_ref_down", 3.5);       // v_ref 하강 한계 [m/s^2]
    v_deadband_    = declare_parameter<double>("v_deadband", 0.15);      // |v_ref - v| < dead → 0 처리

    // ★ TTC 필터 파라미터(정면 시야/복도폭)
    ttc_fov_deg_       = declare_parameter<double>("ttc_fov_deg", 20.0);   // 정면 각도창 ±deg
    ttc_corridor_half_ = declare_parameter<double>("ttc_corridor_half", 0.45); // 복도 반폭[m]

    // ★ 코너 완화 노브(곡률 가중치) — 1.0=기존, 0.85≈코너 더 빠르게, 1.2≈더 느리게
    curv_gain_speed_ = declare_parameter<double>("curv_gain_speed", 1.0);

    // ----- Pub/Sub -----
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

    sub_ttc_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        ttc_scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
          last_ttc_scan_ = *msg; has_ttc_scan_ = true;
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&PurePursuitNode::onTimer, this));
  }

private:
  // ★ PI 상수(안전하게 직접 정의)
  static constexpr double kPI() { return 3.14159265358979323846; }

  // ★ TTC 계산: 정면 각도창 + 복도폭 필터 적용
  static double computeMinTTC(const sensor_msgs::msg::LaserScan& s,
                              double v,
                              double fov_rad,          // 정면 허용각(라디안)
                              double corridor_half) {  // 복도 반폭[m]
    if (v < 0.05) return 1e9;
    double ang = s.angle_min, min_ttc = 1e9;
    for (size_t i=0;i<s.ranges.size(); ++i, ang += s.angle_increment){
      const float r = s.ranges[i];
      if (!std::isfinite(r)) continue;

      // (1) 정면 각도창 필터
      if (std::abs(ang) > fov_rad) continue;

      // (2) 복도 필터: 빔 끝점 횡오프셋 제한
      const double y = r * std::sin(ang);
      if (std::abs(y) > corridor_half) continue;

      // (3) 전방 상대속도
      const double v_rel = v * std::cos(ang);
      if (v_rel <= 0) continue;

      const double ttc = r / std::max(1e-3, v_rel);
      if (ttc < min_ttc) min_ttc = ttc;
    }
    return min_ttc;
  }

  // 1차 LPF
  static double lpf(double x, double x_prev, double tau, double dt){
    const double a = dt/(tau+dt);
    return x_prev + a*(x - x_prev);
  }

  // v_ref 상승/하강 래이트 제한(비대칭)
  static double slew(double target, double prev, double up, double down, double dt){
    const double dv = target - prev;
    const double max_step = (dv >= 0.0 ? up*dt : down*dt);
    if (std::abs(dv) > std::abs(max_step)) {
      return prev + std::copysign(std::abs(max_step), dv);
    }
    return target;
  }

  void onTimer() {
    if (!has_odom_ || center_path_.poses.empty()) return;

    // 현재 포즈/자세
    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    const double x = p.x, y = p.y;
    const double v = odom_.twist.twist.linear.x;

    // 동적 룩어헤드
    const double Ld_target = std::clamp(ld0_ + kv_ld_ * v, ld_min_, ld_max_);

    // 타깃 인덱스
    const int target_idx = findLookaheadIndex(center_path_, x, y, Ld_target);
    if (target_idx < 0) return;

    const auto &tp = center_path_.poses[target_idx].pose.position;

    // 차량 좌표계로 변환(x: 전방, y: 좌측)
    const double dx = tp.x - x;
    const double dy = tp.y - y;
    const double xL =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (xL <= 0.01) return;

    // PP 기하
    const double Ld = std::hypot(xL, yL);
    const double curvature = 2.0 * yL / (Ld * Ld);
    const double steer = std::atan(wheelbase_ * curvature);

    // (1) 곡률 기반 속도 상한 (ay <= ay_max)  ← ★ curv_gain_speed_ 반영
    const double kappa_eff = std::max(1e-6, curv_gain_speed_ * std::abs(curvature));
    double v_curv = (kappa_eff < 1e-4) ? v_max_
               : std::sqrt(std::max(1e-6, ay_max_ / kappa_eff));
    v_curv = std::clamp(v_curv, v_min_, v_max_);

    // (2) TTC 기반 속도 상한 (게이트 + 필터 적용)
    double v_ttc = v_max_;
    if (has_ttc_scan_) {
      const double ttc_min = computeMinTTC(
        last_ttc_scan_,
        std::max(0.0, v),
        ttc_fov_deg_ * kPI() / 180.0,   // ★ deg→rad
        ttc_corridor_half_);

      if (ttc_min < ttc_gate_) {
        v_ttc = std::clamp(k_ttc_ * (ttc_min - ttc_safe_), 0.0, v_max_);
      } else {
        v_ttc = v_max_; // 위협 없으면 TTC 비활성
      }
    }

    // (3) 최종 속도 참조: min(곡률, TTC) → LPF → 래이트 제한
    const double v_ref_raw = std::min(v_curv, v_ttc);
    const double v_ref_lpf = lpf(v_ref_raw, v_cmd_prev_, tau_v_, 0.01);        // 10ms 주기
    const double v_ref     = slew(v_ref_lpf, v_cmd_prev_, a_ref_up_, a_ref_down_, 0.01);
    v_cmd_prev_ = v_ref;

    // (4) 데드밴드 + P제어 + 포화
    double err = v_ref - v;
    if (std::abs(err) < v_deadband_) err = 0.0;

    double a_cmd = k_accel_ * err;
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;

    // 퍼블리시 (acceleration 모드)
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.acceleration   = a_cmd;
    cmd.drive.speed          = 0.0;     // 가속 모드 고정

    drive_pub_->publish(cmd);

    // 디버그(선택): 0.5s마다 수치 확인
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "v=%.2f v_ref=%.2f (curv=%.2f ttcCap=%.2f) a=%.2f Ld=%.2f",
      v, v_ref, v_curv, v_ttc, a_cmd, Ld_target);
  }

  // Ld 이상 떨어진 첫 점 찾기
  static int findLookaheadIndex(const nav_msgs::msg::Path &path,
                                double x, double y, double Ld) {
    int closest = 0;
    double best = 1e18;
    for (size_t i=0; i<path.poses.size(); ++i) {
      const auto &pt = path.poses[i].pose.position;
      const double d2 = (pt.x - x)*(pt.x - x) + (pt.y - y)*(pt.y - y);
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

  // ----- 멤버 -----
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_center_, sub_left_, sub_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ttc_scan_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path center_path_, left_path_, right_path_;
  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::LaserScan last_ttc_scan_;
  bool has_odom_{false}, has_ttc_scan_{false};

  // ----- 파라미터/상태 -----
  double lookahead_, wheelbase_;
  double v_min_, v_max_, k_speed_;
  double k_accel_, a_min_, a_max_;
  std::string center_path_topic_, left_path_topic_, right_path_topic_, odom_topic_, drive_topic_;

  double ay_max_, ld0_, kv_ld_, ld_min_, ld_max_;
  double ttc_safe_, k_ttc_, tau_v_, ttc_gate_;
  std::string ttc_scan_topic_;

  double a_ref_up_, a_ref_down_, v_deadband_;
  double v_cmd_prev_{0.0};

  // ★ TTC 필터 파라미터
  double ttc_fov_deg_, ttc_corridor_half_;

  // ★ 코너 완화 노브
  double curv_gain_speed_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}

