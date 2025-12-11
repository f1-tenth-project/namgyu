// pure_pursuit.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/marker.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>   // 벽 RANSAC용 난수
#include <limits>   // numeric_limits
#include <chrono>   // timer duration

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    // ----- 기본 파라미터 -----
    lookahead_ = declare_parameter<double>("lookahead", 0.8);     // [m] 기본 Ld (저속 Ld)
    wheelbase_ = declare_parameter<double>("wheelbase", 0.34);    // [m]
    v_min_     = declare_parameter<double>("speed_min", 1.0);     // [m/s]
    v_max_     = declare_parameter<double>("speed_max", 20.0);    // [m/s]
    k_accel_   = declare_parameter<double>("k_accel",  2.5);      // P gain for (v_ref - v)
    a_min_     = declare_parameter<double>("accel_min",-9.0);     // [m/s^2]
    a_max_     = declare_parameter<double>("accel_max",  4.0);    // [m/s^2]

    // ----- 동적 룩어헤드 파라미터 -----
    kv_ld_      = declare_parameter<double>("kv_ld",   0.3);      // 속도 비례 계수
    ld_min_     = declare_parameter<double>("ld_min",  0.6);      // 최소 Ld
    ld_max_     = declare_parameter<double>("ld_max",  4.0);      // 최대 Ld
    k_ld_curv_  = declare_parameter<double>("k_ld_curv", 4.0);    // 곡률 기반 Ld 축소 강도

    // ----- 동적 곡률 프리뷰 파라미터 -----
    curv_preview_t_    = declare_parameter<double>("curv_preview_time", 0.8);  // [s]
    curv_preview_min_  = declare_parameter<double>("curv_preview_min",  3.0);  // [m]
    curv_preview_max_  = declare_parameter<double>("curv_preview_max",  9.0);  // [m]
    curv_preview_step_ = declare_parameter<double>("curv_preview_step", 0.3);  // [m]

    // 곡률 기반 속도 제한용 파라미터
    ay_max_          = declare_parameter<double>("ay_max",          3.0);  // [m/s^2]
    curv_gain_speed_ = declare_parameter<double>("curv_gain_speed", 1.5);  // 곡률 민감도

    // ----- 경로/토픽 파라미터 -----
    center_path_topic_ = declare_parameter<std::string>("center_path_topic", "center_path");
    left_path_topic_   = declare_parameter<std::string>("left_boundary",   "left_boundary");
    right_path_topic_  = declare_parameter<std::string>("right_boundary",  "right_boundary");
    odom_topic_        = declare_parameter<std::string>("odom_topic",      "odom0");
    drive_topic_       = declare_parameter<std::string>("drive_topic",     "ackermann_cmd0");

    // raceline CSV 경로
    center_path_csv_ = declare_parameter<std::string>(
        "center_path_csv",
        "/home/namgyu/race/src/racecar_simulator/maps/lane_process/icra2025/icra2025_raceline.csv"
    );

    // ----- LiDAR / 군집 기반 장애물 인식 파라미터 -----
    scan_topic_      = declare_parameter<std::string>("scan_topic",      "scan0");
    scan_frame_id_   = declare_parameter<std::string>("scan_frame_id",   "base_link0");

    obs_fov_deg_        = declare_parameter<double>("obs_fov_deg",        60.0); // 정면 ±60도
    obs_corridor_half_  = declare_parameter<double>("obs_corridor_half",  0.6);  // 복도 반폭 [m]
    obs_x_min_          = declare_parameter<double>("obs_x_min",          0.3);  // 차 바로 앞은 제외
    obs_max_range_      = declare_parameter<double>("obs_max_range",      8.0);  // 최대 사용 거리
    obs_cluster_dist_   = declare_parameter<double>("obs_cluster_dist",   0.35); // 군집 내부 점 간 거리 임계
    obs_min_points_     = declare_parameter<int>("obs_min_points",        3);    // 최소 포인트 수
    obs_front_y_thresh_ = declare_parameter<double>("obs_front_y_thresh", 0.4);  // 정면 범위 |cy| <= 이 값

    // ----- 벽 RANSAC 파라미터 -----
    wall_inlier_thresh_      = declare_parameter<double>("wall_inlier_thresh", 0.07);  // [m]
    wall_ransac_iters_       = declare_parameter<int>("wall_ransac_iters", 80);
    wall_ransac_min_inliers_ = declare_parameter<int>("wall_ransac_min_inliers", 20);

    // ----- 트랙 기반 장애물 필터 파라미터 -----
    track_wall_margin_     = declare_parameter<double>("track_wall_margin", 0.4);
    track_center_max_dist_ = declare_parameter<double>("track_center_max_dist", 1.2);

    // ----- 장애물 감속 파라미터 -----
    obstacle_slow_dist_ = declare_parameter<double>("obstacle_slow_dist", 5.0);
    obstacle_stop_dist_ = declare_parameter<double>("obstacle_stop_dist", 2.0);
    obstacle_v_min_     = declare_parameter<double>("obstacle_v_min",     0.5);

    // ----- 회피용 PID & 스무딩 파라미터 -----
    k_p_avoid_ = declare_parameter<double>("k_p_avoid", 0.8);
    k_i_avoid_ = declare_parameter<double>("k_i_avoid", 0.0);
    k_d_avoid_ = declare_parameter<double>("k_d_avoid", 0.1);

    steering_lpf_alpha_ = declare_parameter<double>("steer_lpf_alpha", 0.6); // 0.3~0.7
    max_steer_rate_     = declare_parameter<double>("max_steer_rate", 2.0);  // [rad/s]
    steer_ftg_max_      = declare_parameter<double>("steer_ftg_max", 0.7);   // FTG 최대 조향 [rad]

    // ----- Publisher / Subscriber -----
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    raceline_pub_ = create_publisher<nav_msgs::msg::Path>("raceline_path", 10);
    lookahead_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);
    curvature_pub_ = create_publisher<visualization_msgs::msg::Marker>("curvature_points", 10);
    obstacle_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("obstacle_clusters", 10);
    wall_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("pp_debug/walls", 10);

    sub_center_ = create_subscription<nav_msgs::msg::Path>(
        center_path_topic_, 1,
        [this](nav_msgs::msg::Path::SharedPtr msg){ center_path_from_topic_ = *msg; });

    sub_left_ = create_subscription<nav_msgs::msg::Path>(
        left_path_topic_, 1,
        [this](nav_msgs::msg::Path::SharedPtr msg){ left_path_ = *msg; });

    sub_right_ = create_subscription<nav_msgs::msg::Path>(
        right_path_topic_, 1,
        [this](nav_msgs::msg::Path::SharedPtr msg){ right_path_ = *msg; });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg){
          odom_ = *msg; has_odom_ = true;
        });

    // LiDAR 스캔 구독 → 군집 + 벽 인식 + FTG 타겟
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
          detectObstacleClusters(*msg);
        });

    // ----- CSV 에서 raceline 로드 -----
    loadCenterPathFromCsv(center_path_csv_);

    // 주행 타이머 (10ms)
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PurePursuitNode::onTimer, this));

    // RANSAC 난수 시드
    rng_.seed(12345);
  }

private:
  // ======================= CSV 로 경로 로드 =======================
  void loadCenterPathFromCsv(const std::string &csv_path) {
    center_path_.poses.clear();
    center_path_.header.frame_id = "map";

    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open center_path csv: %s", csv_path.c_str());
      return;
    }

    std::string line;
    if (!std::getline(ifs, line)) {
      RCLCPP_ERROR(this->get_logger(),
                   "center_path csv is empty: %s", csv_path.c_str());
      return;
    }

    int count = 0;
    while (std::getline(ifs, line)) {
      if (line.empty()) continue;
      std::stringstream ss(line);
      std::string sx, sy;
      if (!std::getline(ss, sx, ',')) continue;
      if (!std::getline(ss, sy, ',')) continue;

      try {
        double x = std::stod(sx);
        double y = std::stod(sy);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;

        center_path_.poses.push_back(ps);
        ++count;
      } catch (...) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to parse line in center_path csv: '%s'", line.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded center_path (raceline) from csv: %s (points=%d)",
                csv_path.c_str(), count);
  }

  // ======================= 메인 주행 루프 =======================
  void onTimer() {
    if (!has_odom_ || center_path_.poses.empty()) return;

    center_path_.header.frame_id = "map";
    center_path_.header.stamp = now();
    raceline_pub_->publish(center_path_);

    // ----- 현재 자세 -----
    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    const double x = p.x, y = p.y;

    // 현재 속도
    const double v = odom_.twist.twist.linear.x;

    // ----- 동적 곡률 프리뷰 -----
    double L_curv = curv_preview_min_ + curv_preview_t_ * std::max(0.0, v);
    L_curv = std::clamp(L_curv, curv_preview_min_, curv_preview_max_);

    std::vector<geometry_msgs::msg::Point> preview_points;
    double kappa_ahead = computeMaxCurvatureAhead(center_path_, x, y,
                                                  L_curv, curv_preview_step_,
                                                  preview_points);
    publishCurvaturePointsMarker(preview_points);

    // ----- 동적 룩어헤드 -----
    double Ld_base = lookahead_ + kv_ld_ * std::max(0.0, v);
    double kappa_abs = std::abs(kappa_ahead);
    double curv_factor = 1.0 / (1.0 + k_ld_curv_ * kappa_abs);
    curv_factor = std::max(curv_factor, 0.3);   // 최소 0.3배
    double Ld = std::clamp(Ld_base * curv_factor, ld_min_, ld_max_);

    int target_idx = findLookaheadIndex(center_path_, x, y, Ld);
    if (target_idx < 0) return;
    const auto &tp = center_path_.poses[target_idx].pose.position;

    const double dx = tp.x - x;
    const double dy = tp.y - y;
    const double xL =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (xL <= 0.01) return;

    const double curvature_pp = 2.0 * yL / (Ld * Ld);
    double steer_pp = std::atan(wheelbase_ * curvature_pp);

    publishLookaheadMarker(tp);

    // ----- 곡률 기반 속도 상한 -----
    double kappa_eff = std::max(1e-6, curv_gain_speed_ * std::abs(kappa_ahead));
    double v_curv = std::sqrt(ay_max_ / kappa_eff);
    double v_ref  = std::clamp(v_curv, v_min_, v_max_);

    // ----- 장애물 거리 LPF -----
    double d_obs = nearest_obstacle_dist_;
    double alpha_d = 0.5; // 0.3~0.7 사이에서 튜닝
    if (std::isfinite(d_obs)) {
      if (!std::isfinite(nearest_obstacle_dist_filt_)) {
        nearest_obstacle_dist_filt_ = d_obs;
      } else {
        nearest_obstacle_dist_filt_ =
            alpha_d * d_obs + (1.0 - alpha_d) * nearest_obstacle_dist_filt_;
      }
    } else {
      nearest_obstacle_dist_filt_ = std::numeric_limits<double>::infinity();
    }
    d_obs = nearest_obstacle_dist_filt_;

    // ===== 장애물 기반 감속 (곡선형, LPF 거리 사용 + 히스테리시스 플래그 사용) =====
    if (has_obstacle_stable_ && d_obs < obstacle_slow_dist_) {
      double d = d_obs;
      double v_lim = v_ref;

      if (d <= obstacle_stop_dist_) {
        v_lim = obstacle_v_min_;
      } else {
        double s = (d - obstacle_stop_dist_) /
                   (obstacle_slow_dist_ - obstacle_stop_dist_);
        s = std::clamp(s, 0.0, 1.0);
        double w = s * s; // 멀수록 1, 가까울수록 0로 급하게
        v_lim = obstacle_v_min_ + w * (v_ref - obstacle_v_min_);
      }
      v_ref = std::min(v_ref, v_lim);
    }

    // ----- v_ref LPF (속도 목표도 부드럽게) -----
    double v_ref_raw = v_ref;
    double alpha_v = 0.3; // 0.2~0.5 정도
    if (!v_ref_init_) {
      v_ref_filt_ = v_ref_raw;
      v_ref_init_ = true;
    } else {
      v_ref_filt_ = alpha_v * v_ref_raw + (1.0 - alpha_v) * v_ref_filt_;
    }

    // ===== FTG + PID + 스무딩 조향 =====
    const double dt = 0.01; // 타이머 10ms

    double steer_cmd = steer_pp;

    // 장애물 거리 기반 FTG 가중치 (LPF 거리 & 히스테리시스 사용)
    double w_ftg = 0.0;
    if (has_obstacle_stable_ && has_ftg_target_) {
      double d = d_obs;
      if (d <= obstacle_stop_dist_) {
        w_ftg = 1.0;
      } else if (d >= obstacle_slow_dist_) {
        w_ftg = 0.0;
      } else {
        double s = (obstacle_slow_dist_ - d) /
                   (obstacle_slow_dist_ - obstacle_stop_dist_);
        s = std::clamp(s, 0.0, 1.0);
        // ★ 더 빨리 FTG 비중이 올라가도록 s^2 대신 s 사용
        w_ftg = s;
      }
    }

    if (w_ftg > 1e-3) {
      double e = steer_ftg_ - steer_pp;

      avoid_err_int_ += e * dt;
      double i_max = 0.3;
      avoid_err_int_ = std::clamp(avoid_err_int_, -i_max, i_max);

      double de = (e - avoid_err_prev_) / dt;
      avoid_err_prev_ = e;

      double delta = k_p_avoid_ * e + k_i_avoid_ * avoid_err_int_ + k_d_avoid_ * de;

      steer_cmd = steer_pp + w_ftg * delta;
    } else {
      avoid_err_int_  = 0.0;
      avoid_err_prev_ = 0.0;
      steer_cmd = steer_pp;
    }

    // ----- 조향 변화율 제한 (장애물 가까우면 완화) -----
    double max_rate = max_steer_rate_;
    if (has_obstacle_stable_ && has_ftg_target_ && d_obs < obstacle_slow_dist_) {
      // 장애물에 가까울수록 더 빨리 틀 수 있게 최대 조향 속도를 1~3배로 스케일
      double g = (obstacle_slow_dist_ - d_obs) /
                 (std::max(1e-3, obstacle_slow_dist_ - obstacle_stop_dist_));
      g = std::clamp(g, 0.0, 1.0);
      max_rate = max_steer_rate_ * (1.0 + 2.0 * g);  // max 3배
    }

    double max_delta = max_rate * dt;
    double delta_raw = steer_cmd - steer_prev_;
    delta_raw = std::clamp(delta_raw, -max_delta, max_delta);
    double steer_step = steer_prev_ + delta_raw;

    // 1차 LPF
    double steer_final = steering_lpf_alpha_ * steer_step
                       + (1.0 - steering_lpf_alpha_) * steer_prev_;
    steer_prev_ = steer_final;

    // ----- 가속도 명령 -----
    double a_cmd = k_accel_ * (v_ref_filt_ - v); // 필터된 v_ref 사용
    a_cmd = std::clamp(a_cmd, a_min_, a_max_);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer_final;
    cmd.drive.acceleration   = a_cmd;

    drive_pub_->publish(cmd);
  }

  // ======================= 장애물 군집 + 벽 인식 =======================
  struct ObstacleCluster {
    std::vector<geometry_msgs::msg::Point> pts;
    double cx{0.0};
    double cy{0.0};
    double r_min{1e9};
  };

  struct Point2D {
    double x;
    double y;
  };

  struct LineModel {
    double a{0.0}, b{0.0}, c{0.0}; // ax + by + c = 0
    bool valid{false};
  };

  void detectObstacleClusters(const sensor_msgs::msg::LaserScan &scan)
  {
    clusters_.clear();

    const double fov_rad = obs_fov_deg_ * M_PI / 180.0;
    const double y_half  = obs_corridor_half_;
    const double x_min   = obs_x_min_;
    const double max_r   = obs_max_range_;
    const double th_dist = obs_cluster_dist_;
    const int    min_pts = std::max(1, obs_min_points_);

    // ---------- 1) 벽 후보 점 수집 ----------
    std::vector<Point2D> left_wall_pts;
    std::vector<Point2D> right_wall_pts;
    left_wall_pts.reserve(scan.ranges.size() / 2);
    right_wall_pts.reserve(scan.ranges.size() / 2);

    double ang = scan.angle_min;
    for (size_t i=0; i<scan.ranges.size(); ++i, ang += scan.angle_increment) {
      float r = scan.ranges[i];
      if (!std::isfinite(r) || r <= 0.05f) continue;
      if (r > (float)max_r) r = (float)max_r;
      if (std::abs(ang) > fov_rad) continue;

      double x = r * std::cos(ang);
      double y = r * std::sin(ang);
      if (x < x_min) continue;

      Point2D p{x, y};
      if (y >= 0.0)
        left_wall_pts.push_back(p);
      else
        right_wall_pts.push_back(p);
    }

    // ---------- 2) RANSAC으로 좌/우 벽 직선 추정 ----------
    LineModel left_wall, right_wall;
    fitWallRANSAC(left_wall_pts, left_wall);
    fitWallRANSAC(right_wall_pts, right_wall);

    // ---------- 3) 벽 제외하면서 군집 클러스터링 ----------
    std::vector<ObstacleCluster> clusters_tmp;
    std::vector<geometry_msgs::msg::Point> cur_pts;
    bool have_cluster = false;
    double prev_x = 0.0, prev_y = 0.0;

    ang = scan.angle_min;

    auto flush_cluster = [&]() {
      if (!have_cluster) return;
      if ((int)cur_pts.size() < min_pts) {
        cur_pts.clear();
        have_cluster = false;
        return;
      }

      ObstacleCluster c;
      c.pts = cur_pts;
      double cx = 0.0, cy = 0.0;
      double rmin = 1e9;
      for (auto &pt : cur_pts) {
        cx += pt.x;
        cy += pt.y;
        double r = std::hypot(pt.x, pt.y);
        if (r < rmin) rmin = r;
      }
      c.cx = cx / (double)cur_pts.size();
      c.cy = cy / (double)cur_pts.size();
      c.r_min = (cur_pts.empty() ? 1e9 : rmin);
      clusters_tmp.push_back(c);

      cur_pts.clear();
      have_cluster = false;
    };

    for (size_t i=0; i<scan.ranges.size(); ++i, ang += scan.angle_increment) {
      float r = scan.ranges[i];

      bool valid = true;
      if (!std::isfinite(r) || r <= 0.05f) valid = false;
      if (valid && r > (float)max_r) r = (float)max_r;
      if (valid && std::abs(ang) > fov_rad) valid = false;

      double x=0.0, y=0.0;
      if (valid) {
        x = r * std::cos(ang);
        y = r * std::sin(ang);

        if (x < x_min) valid = false;
        if (std::abs(y) > y_half) valid = false;  // 복도 밖(사이드)은 장애물 제외

        // 벽 직선에 너무 가까우면 장애물에서 제외
        if (valid) {
          Point2D p{x,y};
          bool is_wall = false;
          if (left_wall.valid && pointLineDistance(left_wall, p) < wall_inlier_thresh_)
            is_wall = true;
          if (right_wall.valid && pointLineDistance(right_wall, p) < wall_inlier_thresh_)
            is_wall = true;
          if (is_wall) valid = false;
        }
      }

      if (!valid) {
        if (have_cluster) {
          flush_cluster();
        }
        continue;
      }

      geometry_msgs::msg::Point pt;
      pt.x = x; pt.y = y; pt.z = 0.0;

      if (!have_cluster) {
        cur_pts.clear();
        cur_pts.push_back(pt);
        prev_x = x; prev_y = y;
        have_cluster = true;
      } else {
        double d = std::hypot(x - prev_x, y - prev_y);
        if (d > th_dist) {
          flush_cluster();
          cur_pts.clear();
          cur_pts.push_back(pt);
        } else {
          cur_pts.push_back(pt);
        }
        prev_x = x; prev_y = y;
        have_cluster = true;
      }
    }
    flush_cluster();

    // ---------- 3-1) 트랙 기반 필터링 ----------
    clusters_.clear();
    for (const auto &c : clusters_tmp) {
      if (isClusterInsideTrack(c)) {
        clusters_.push_back(c);
      }
    }

    // ===== 장애물 최소 거리 계산 =====
    nearest_obstacle_dist_ = std::numeric_limits<double>::infinity();
    for (const auto &c : clusters_) {
      if (std::abs(c.cy) > obs_front_y_thresh_) continue;
      double d = c.r_min;
      if (d < nearest_obstacle_dist_) {
        nearest_obstacle_dist_ = d;
      }
    }
    has_obstacle_ = std::isfinite(nearest_obstacle_dist_);

    // ----- 장애물 히스테리시스 (깜빡임 방지) -----
    if (!has_obstacle_stable_) {
      // 꺼져 있을 때: 슬로우 구간 안까지 들어와야 ON
      if (has_obstacle_ && nearest_obstacle_dist_ < obstacle_slow_dist_) {
        has_obstacle_stable_ = true;
      }
    } else {
      // 켜져 있을 때: 충분히 멀어지거나 장애물이 사라져야 OFF
      if (!has_obstacle_ || nearest_obstacle_dist_ > obstacle_slow_dist_ + 1.0) {
        has_obstacle_stable_ = false;
      }
    }

    // ---------- 마커들 ----------
    publishObstacleMarkers(clusters_);
    publishWallMarkers(left_wall, right_wall);

    // ---------- FTG 타겟 계산 ----------
    computeFTGTargetFromScan(scan);
  }

  // ===== FTG 타겟 조향 계산 (정면 장애물 시 좌/우 갭 선호) =====
  void computeFTGTargetFromScan(const sensor_msgs::msg::LaserScan &scan)
  {
    has_ftg_target_ = false;
    steer_ftg_ = 0.0;

    if (scan.ranges.empty()) return;

    double fov_rad = obs_fov_deg_ * M_PI / 180.0;

    // 1) 먼저 FOV 안에서 최소 거리(min_r)를 구해서 "가까운 장애물 모드" 여부 결정
    double min_r = std::numeric_limits<double>::infinity();
    double ang = scan.angle_min;
    for (size_t i=0; i<scan.ranges.size(); ++i, ang += scan.angle_increment) {
      float r = scan.ranges[i];
      if (std::abs(ang) > fov_rad) continue;
      if (!std::isfinite(r) || r <= 0.05f) continue;
      if (r > (float)obs_max_range_) r = (float)obs_max_range_;
      if (r < min_r) min_r = r;
    }

    if (!std::isfinite(min_r)) {
      // 유효한 포인트가 없음
      return;
    }

    const double close_thresh = 3.0;     // 이 거리 안이면 "가까운 장애물"로 판단
    const bool   near_mode    = (min_r < close_thresh);
    const double lateral_gain = 2.0;     // 좌/우로 벌어진 방향을 선호하는 가중치

    // 2) 점수 기반으로 최적 각도 선택
    double best_score = -1e9;
    double best_r  = -1.0;
    double best_th = 0.0;

    ang = scan.angle_min;
    for (size_t i=0; i<scan.ranges.size(); ++i, ang += scan.angle_increment) {
      float r = scan.ranges[i];
      if (std::abs(ang) > fov_rad) continue;
      if (!std::isfinite(r) || r <= 0.05f) continue;

      if (r > (float)obs_max_range_) r = (float)obs_max_range_;

      // 기본은 "더 멀리 뚫린 방향" 선호
      double score = r;

      if (near_mode) {
        // 가까운 장애물이 있을 때는 정면(ang≈0)을 피하고
        // 좌/우로 벌어진 방향(|ang| 큰 곳)을 추가 가점
        double norm_ang = std::abs(ang) / fov_rad; // 0~1
        score += lateral_gain * norm_ang;
      }

      if (score > best_score) {
        best_score = score;
        best_r     = r;
        best_th    = ang;  // base_link 기준 각도
      }
    }

    if (best_r <= 0.0) {
      has_ftg_target_ = false;
      return;
    }

    // Pure Pursuit 형식으로 FTG 타겟 조향 계산
    double r = std::min(best_r, 4.0);  // 너무 멀면 Ld 상한 4m 정도
    double y = r * std::sin(best_th);

    if (r < 0.5) {
      // 너무 가까운 점은 사용하지 않음
      has_ftg_target_ = false;
      return;
    }

    double kappa = 2.0 * y / (r * r);
    double steer = std::atan(wheelbase_ * kappa);

    // FTG 조향 한도
    steer = std::clamp(steer, -steer_ftg_max_, steer_ftg_max_);

    steer_ftg_ = steer;
    has_ftg_target_ = true;
  }

  // 군집 중심 마커 (장애물)
  void publishObstacleMarkers(const std::vector<ObstacleCluster> &clusters)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = scan_frame_id_;
    mk.header.stamp = now();
    mk.ns = "obstacle_clusters";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.scale.x = 0.25;
    mk.scale.y = 0.25;
    mk.scale.z = 0.25;

    mk.color.r = 1.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.points.clear();

    for (const auto &c : clusters) {
      if (std::abs(c.cy) > obs_front_y_thresh_) continue;

      geometry_msgs::msg::Point p;
      p.x = c.cx;
      p.y = c.cy;
      p.z = 0.0;
      mk.points.push_back(p);
    }

    if (mk.points.empty()) {
      mk.action = visualization_msgs::msg::Marker::DELETE;
    }

    mk.lifetime = rclcpp::Duration::from_seconds(0.1);
    obstacle_marker_pub_->publish(mk);
  }

  // ===== 벽 RANSAC 보조 =====
  static double pointLineDistance(const LineModel &line, const Point2D &p) {
    if (!line.valid) return 1e9;
    return std::fabs(line.a * p.x + line.b * p.y + line.c) /
           std::sqrt(line.a*line.a + line.b*line.b);
  }

  void fitWallRANSAC(const std::vector<Point2D> &pts, LineModel &best_model) {
    best_model.valid = false;
    if (pts.size() < 2) return;

    const int max_iters   = wall_ransac_iters_;
    const int min_inliers = wall_ransac_min_inliers_;

    std::uniform_int_distribution<int> dist_idx(0, (int)pts.size() - 1);

    int best_inliers = 0;

    for (int it = 0; it < max_iters; ++it) {
      int i1 = dist_idx(rng_);
      int i2 = dist_idx(rng_);
      if (i1 == i2) continue;

      const auto &p1 = pts[i1];
      const auto &p2 = pts[i2];

      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      if (std::hypot(dx, dy) < 1e-3) continue;

      LineModel candidate;
      candidate.a =  dy;
      candidate.b = -dx;
      candidate.c = -(candidate.a * p1.x + candidate.b * p1.y);
      candidate.valid = true;

      int inliers = 0;
      for (const auto &p : pts) {
        double d = pointLineDistance(candidate, p);
        if (d < wall_inlier_thresh_) ++inliers;
      }

      if (inliers > best_inliers) {
        best_inliers = inliers;
        best_model = candidate;
        best_model.valid = true;
      }
    }

    if (!best_model.valid || best_inliers < min_inliers) {
      best_model.valid = false;
    }
  }

  // 벽 마커 (보라색 LINE_LIST) publish
  void publishWallMarkers(const LineModel &left_wall, const LineModel &right_wall) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = scan_frame_id_;
    mk.header.stamp = now();
    mk.ns = "walls";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.scale.x = 0.05;

    mk.color.r = 1.0f;
    mk.color.g = 0.0f;
    mk.color.b = 1.0f;
    mk.color.a = 1.0f;

    mk.points.clear();

    auto add_line = [&](const LineModel &line) {
      if (!line.valid) return;
      double x0 = obs_x_min_;
      double x1 = obs_max_range_;

      if (std::fabs(line.b) < 1e-6) return;

      geometry_msgs::msg::Point p1, p2;
      p1.x = x0;
      p1.y = -(line.a * x0 + line.c) / line.b;
      p1.z = 0.0;

      p2.x = x1;
      p2.y = -(line.a * x1 + line.c) / line.b;
      p2.z = 0.0;

      mk.points.push_back(p1);
      mk.points.push_back(p2);
    };

    add_line(left_wall);
    add_line(right_wall);

    if (mk.points.empty()) {
      mk.action = visualization_msgs::msg::Marker::DELETE;
    }

    mk.lifetime = rclcpp::Duration::from_seconds(0.1);
    wall_marker_pub_->publish(mk);
  }

  // ======================= 트랙 기반 장애물 필터 =======================
  bool clusterCenterToMap(const ObstacleCluster &c, double &X, double &Y) const
  {
    if (!has_odom_) return false;

    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;

    double roll, pitch, yaw;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);

    double x = c.cx;
    double y = c.cy;

    X = p.x + std::cos(yaw) * x - std::sin(yaw) * y;
    Y = p.y + std::sin(yaw) * x + std::cos(yaw) * y;
    return true;
  }

  double distanceToPath(const nav_msgs::msg::Path &path, double X, double Y) const
  {
    if (path.poses.empty()) return 1e9;

    double best2 = 1e18;
    for (const auto &ps : path.poses) {
      double dx = ps.pose.position.x - X;
      double dy = ps.pose.position.y - Y;
      double d2 = dx*dx + dy*dy;
      if (d2 < best2) best2 = d2;
    }
    return std::sqrt(best2);
  }

  bool isClusterInsideTrack(const ObstacleCluster &c) const
  {
    if (left_path_.poses.empty() || right_path_.poses.empty()) {
      return true;
    }

    double X, Y;
    if (!clusterCenterToMap(c, X, Y)) {
      return false;
    }

    double dL = distanceToPath(left_path_,  X, Y);
    double dR = distanceToPath(right_path_, X, Y);
    double d_min = std::min(dL, dR);

    if (d_min < track_wall_margin_) {
      return false;
    }

    const nav_msgs::msg::Path *center_for_class = nullptr;
    if (!center_path_from_topic_.poses.empty()) {
      center_for_class = &center_path_from_topic_;
    } else if (!center_path_.poses.empty()) {
      center_for_class = &center_path_;
    }

    if (center_for_class) {
      double dC = distanceToPath(*center_for_class, X, Y);
      if (dC > track_center_max_dist_) {
        return false;
      }
    }

    return true;
  }

  // ======================= 마커/유틸 =======================
  void publishLookaheadMarker(const geometry_msgs::msg::Point &pt) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = now();
    mk.ns = "lookahead_point";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose.position = pt;
    mk.pose.position.z = 0.1;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = 0.3;
    mk.scale.y = 0.3;
    mk.scale.z = 0.3;

    mk.color.r = 0.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.lifetime = rclcpp::Duration::from_seconds(0.2);
    lookahead_pub_->publish(mk);
  }

  void publishCurvaturePointsMarker(const std::vector<geometry_msgs::msg::Point> &pts) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = now();
    mk.ns = "curvature_points";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.scale.x = 0.25;
    mk.scale.y = 0.25;
    mk.scale.z = 0.25;

    mk.color.r = 1.0f;
    mk.color.g = 0.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.points = pts;
    mk.lifetime = rclcpp::Duration::from_seconds(0.2);
    curvature_pub_->publish(mk);
  }

  static int findLookaheadIndex(const nav_msgs::msg::Path &path,
                                double x, double y, double Ld) {
    if (path.poses.empty()) return -1;

    int closest = 0;
    double best = 1e18;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto &pt = path.poses[i].pose.position;
      double d2 = (pt.x - x)*(pt.x - x) + (pt.y - y)*(pt.y - y);
      if (d2 < best) { best = d2; closest = static_cast<int>(i); }
    }

    double accum = 0.0;
    for (size_t step = 0; step + 1 < path.poses.size(); ++step) {
      int i = (closest + step) % path.poses.size();
      int j = (closest + step + 1) % path.poses.size();
      const auto &a = path.poses[i].pose.position;
      const auto &b = path.poses[j].pose.position;
      accum += std::hypot(b.x - a.x, b.y - a.y);
      if (accum >= Ld) return j;
    }
    return closest;
  }

  double computeMaxCurvatureAhead(const nav_msgs::msg::Path &path,
                                  double x, double y,
                                  double L_curv, double step_spacing,
                                  std::vector<geometry_msgs::msg::Point> &preview_pts)
  {
    (void)step_spacing;
    preview_pts.clear();
    if (path.poses.size() < 3) return 0.0;

    int closest = 0;
    double best = 1e18;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto &pt = path.poses[i].pose.position;
      double d2 = (pt.x - x)*(pt.x - x) + (pt.y - y)*(pt.y - y);
      if (d2 < best) { best = d2; closest = static_cast<int>(i); }
    }

    double accum = 0.0;
    double max_kappa = 0.0;

    preview_pts.push_back(path.poses[closest].pose.position);
    int curr_idx = closest;

    while (accum < L_curv) {
      int next_idx = (curr_idx + 1) % path.poses.size();
      const auto &a = path.poses[curr_idx].pose.position;
      const auto &b = path.poses[next_idx].pose.position;
      double ds = std::hypot(b.x - a.x, b.y - a.y);
      if (ds < 1e-6) break;

      accum += ds;
      curr_idx = next_idx;
      preview_pts.push_back(path.poses[curr_idx].pose.position);

      int n = static_cast<int>(preview_pts.size());
      if (n >= 3) {
        const auto &p1 = preview_pts[n-3];
        const auto &p2 = preview_pts[n-2];
        const auto &p3 = preview_pts[n-1];

        double kappa = computeCurvatureThreePoints(p1, p2, p3);
        if (std::abs(kappa) > std::abs(max_kappa)) {
          max_kappa = kappa;
        }
      }

      if (accum >= L_curv) break;
    }

    return max_kappa;
  }

  static double computeCurvatureThreePoints(const geometry_msgs::msg::Point &p1,
                                            const geometry_msgs::msg::Point &p2,
                                            const geometry_msgs::msg::Point &p3)
  {
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double x3 = p3.x, y3 = p3.y;

    double a = std::hypot(x2 - x1, y2 - y1);
    double b = std::hypot(x3 - x2, y3 - y2);
    double c = std::hypot(x3 - x1, y3 - y1);

    double s = 0.5 * (a + b + c);
    double area2 = s * (s - a) * (s - b) * (s - c);
    if (area2 <= 1e-12) return 0.0;

    double area = std::sqrt(area2);
    double radius = (a * b * c) / (4.0 * area);
    double kappa = 1.0 / radius;

    double vx1 = x2 - x1;
    double vy1 = y2 - y1;
    double vx2 = x3 - x2;
    double vy2 = y3 - y2;
    double cross = vx1 * vy2 - vy1 * vx2;
    if (cross < 0.0) kappa = -kappa;

    return kappa;
  }

  // ----- 멤버 변수 -----
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raceline_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr curvature_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_marker_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_center_, sub_left_, sub_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path center_path_;
  nav_msgs::msg::Path center_path_from_topic_;
  nav_msgs::msg::Path left_path_, right_path_;

  nav_msgs::msg::Odometry odom_;
  bool has_odom_{false};

  double lookahead_, wheelbase_;
  double v_min_, v_max_;
  double k_accel_, a_min_, a_max_;

  double kv_ld_, ld_min_, ld_max_;
  double k_ld_curv_;

  double curv_preview_t_, curv_preview_min_, curv_preview_max_;
  double curv_preview_step_;

  double ay_max_;
  double curv_gain_speed_;

  std::string center_path_topic_, left_path_topic_, right_path_topic_;
  std::string odom_topic_, drive_topic_;
  std::string center_path_csv_;

  std::string scan_topic_;
  std::string scan_frame_id_;

  double obs_fov_deg_;
  double obs_corridor_half_;
  double obs_x_min_;
  double obs_max_range_;
  double obs_cluster_dist_;
  int    obs_min_points_;
  double obs_front_y_thresh_;

  double wall_inlier_thresh_;
  int    wall_ransac_iters_;
  int    wall_ransac_min_inliers_;

  double track_wall_margin_;
  double track_center_max_dist_;

  double obstacle_slow_dist_;
  double obstacle_stop_dist_;
  double obstacle_v_min_;

  bool   has_obstacle_{false};
  double nearest_obstacle_dist_{std::numeric_limits<double>::infinity()};
  double nearest_obstacle_dist_filt_{std::numeric_limits<double>::infinity()};
  bool   has_obstacle_stable_{false};

  std::vector<ObstacleCluster> clusters_;

  // 회피용 FTG / PID / 스무딩
  double steer_ftg_{0.0};
  bool   has_ftg_target_{false};

  double k_p_avoid_{0.0}, k_i_avoid_{0.0}, k_d_avoid_{0.0};
  double avoid_err_int_{0.0};
  double avoid_err_prev_{0.0};

  double steering_lpf_alpha_{1.0};
  double max_steer_rate_{999.0};
  double steer_prev_{0.0};

  double steer_ftg_max_{0.7};

  double v_ref_filt_{0.0};
  bool   v_ref_init_{false};

  std::mt19937 rng_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
