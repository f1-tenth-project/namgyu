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
// ---------- ★ ADDED: 헤비 코너 인식 유틸 ----------
#include <vector>
#include <limits>
#include <random>
#include <numeric>

// ---------- ★ ADDED: 디버그 토픽/마커 ----------
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sstream>
#include <iomanip>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    // ----- 기본 PP / 토픽 파라미터 -----
    lookahead_ = declare_parameter<double>("lookahead", 1.5);           // (동적 Ld가 대신 사용됨)
    wheelbase_ = declare_parameter<double>("wheelbase", 0.34);
    v_min_     = declare_parameter<double>("speed_min", 1.0);
    v_max_     = declare_parameter<double>("speed_max", 9.0);           // 안전 기본값(직선 하드캡)
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

    // ---------- ★ ADDED: 헤비 코너 인식 파라미터 ----------
    corner_enable_         = declare_parameter<bool>("corner_enable", true);     // 코너 시 TTC 비활성
    corner_side_deg_       = declare_parameter<double>("corner_side_deg", 45.0); // 좌/우 섹터 중심각
    ransac_iters_          = declare_parameter<int>("corner_ransac_iters", 120);
    ransac_tol_            = declare_parameter<double>("corner_ransac_tol", 0.05);  // [m]
    min_inliers_           = declare_parameter<int>("corner_min_inliers", 20);
    angle_thresh_deg_      = declare_parameter<double>("corner_angle_thresh_deg", 25.0);
    intersect_max_dist_    = declare_parameter<double>("corner_intersect_max", 3.0);
    circle_use_            = declare_parameter<bool>("corner_circle_use", true);
    circle_R_max_          = declare_parameter<double>("corner_circle_Rmax", 6.0);
    ema_alpha_             = declare_parameter<double>("corner_ema_alpha", 0.35);
    corner_on_cycles_      = declare_parameter<int>("corner_on_cycles", 3);
    corner_off_cycles_     = declare_parameter<int>("corner_off_cycles", 5);

    // ---------- ★ ADDED: 디버그 스위치/프레임 ----------
    debug_markers_ = declare_parameter<bool>("corner_debug_markers", true);
    debug_frame_   = declare_parameter<std::string>("corner_debug_frame", "base_link0");

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

    // ---------- ★ ADDED: 디버그 퍼블리셔 ----------
    pub_corner_active_ = create_publisher<std_msgs::msg::Bool>("/pp_debug/corner_active", 10);
    pub_corner_conf_   = create_publisher<std_msgs::msg::Float32>("/pp_debug/corner_conf", 10);
    pub_ttc_min_       = create_publisher<std_msgs::msg::Float32>("/pp_debug/ttc_min", 10);
    pub_markers_       = create_publisher<visualization_msgs::msg::MarkerArray>("/pp_debug/markers", 10);

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

  // ---------- ★ ADDED: Pratt 원피팅 ----------
  struct Pt { double x,y; };
  struct Line { double a,b,c; }; // ax+by+c=0 (정규화)

  static inline double dot(double ax,double ay,double bx,double by){ return ax*bx+ay*by; }

  static Line makeLine(const Pt& p1, const Pt& p2){
    const double dx = p2.x - p1.x, dy = p2.y - p1.y;
    double a = dy, b = -dx, c = -(a*p1.x + b*p1.y);
    const double n = std::sqrt(a*a+b*b);
    if (n < 1e-9) return {0,1,0};
    a/=n; b/=n; c/=n;
    return {a,b,c};
  }
  static double lineDist(const Line& L, const Pt& p){ return std::abs(L.a*p.x + L.b*p.y + L.c); }

  static bool lineIntersect(const Line& L1, const Line& L2, Pt& out){
    const double det = L1.a*L2.b - L2.a*L1.b;
    if (std::abs(det) < 1e-9) return false;
    out.x = (L2.b*(-L1.c) - L1.b*(-L2.c)) / det;
    out.y = (L1.a*(-L2.c) - L2.a*(-L1.c)) / det;
    return true;
  }
  static double lineAngleDeg(const Line& L1, const Line& L2){
    const double ux=-L1.b, uy=L1.a, vx=-L2.b, vy=L2.a;
    const double c = std::clamp( dot(ux,uy,vx,vy)/(std::hypot(ux,uy)*std::hypot(vx,vy)), -1.0, 1.0);
    return std::abs( std::acos(c) * 180.0 / kPI() );
  }

  // 섹터 포인트 수집(조향각 기준 좌/우)
  static void collectSectorPoints(const sensor_msgs::msg::LaserScan& s,
                                  double steer, double sign,   // sign>0: 안쪽, sign<0: 바깥쪽
                                  double side_fov_deg, double corridor_half,
                                  std::vector<Pt>& out)
  {
    out.clear();
    const double side = side_fov_deg * kPI() / 180.0;
    double ang = s.angle_min;
    for (size_t i=0; i<s.ranges.size(); ++i, ang += s.angle_increment){
      const float r = s.ranges[i];
      if (!std::isfinite(r) || r <= 0.05) continue;
      const double dth = std::atan2(std::sin(ang - steer), std::cos(ang - steer));
      if (sign > 0.0){ // 안쪽(조향 부호쪽)
        if (dth < 0 || std::abs(dth) > side) continue;
      }else{           // 바깥쪽
        if (dth > 0 || std::abs(dth) > side) continue;
      }
      const double y = r * std::sin(dth);
      if (std::abs(y) > corridor_half*1.5) continue;
      Pt p{ r*std::cos(dth), y };
      if (p.x < 0.05) continue;
      out.push_back(p);
    }
  }

  // RANSAC 직선
  static bool ransacLine(const std::vector<Pt>& pts, int iters, double tol,
                         Line& best, std::vector<int>& inliers_idx)
  {
    if ((int)pts.size() < 2) return false;
    std::mt19937 rng(1234567);
    std::uniform_int_distribution<int> uni(0, (int)pts.size()-1);
    size_t best_cnt = 0;
    for (int it=0; it<iters; ++it){
      int i=uni(rng), j=uni(rng);
      if (i==j) continue;
      Line L = makeLine(pts[i], pts[j]);
      size_t cnt = 0;
      for (size_t k=0;k<pts.size();++k) if (lineDist(L, pts[k]) < tol) ++cnt;
      if (cnt > best_cnt){
        best_cnt = cnt; best = L;
      }
    }
    if (best_cnt < 2) return false;
    inliers_idx.clear(); inliers_idx.reserve(best_cnt);
    for (size_t k=0;k<pts.size();++k) if (lineDist(best, pts[k]) < tol) inliers_idx.push_back((int)k);
    return true;
  }

  // Pratt 원피팅 (Chernov)
  static bool fitCirclePratt(const std::vector<Pt>& pts, Pt& center, double& R)
  {
    const int N = (int)pts.size();
    if (N < 3) return false;

    double meanx=0.0, meany=0.0;
    for (const auto& p: pts){ meanx += p.x; meany += p.y; }
    meanx/=N; meany/=N;

    std::vector<double> X(N), Y(N), Z(N);
    for (int i=0;i<N;++i){
      X[i] = pts[i].x - meanx;
      Y[i] = pts[i].y - meany;
      Z[i] = X[i]*X[i] + Y[i]*Y[i];
    }

    double Mxx=0, Myy=0, Mxy=0, Mxz=0, Myz=0, Mzz=0;
    for (int i=0;i<N;++i){
      const double Xi=X[i], Yi=Y[i], Zi=Z[i];
      Mxx += Xi*Xi;  Myy += Yi*Yi;  Mxy += Xi*Yi;
      Mxz += Xi*Zi;  Myz += Yi*Zi;  Mzz += Zi*Zi;
    }
    Mxx/=N; Myy/=N; Mxy/=N; Mxz/=N; Myz/=N; Mzz/=N;

    const double Mz = Mxx + Myy;
    const double Cov_xy = Mxx*Myy - Mxy*Mxy;

    const double A3 = 4.0*Mz;
    const double A2 = -3.0*Mz*Mz - Mzz;
    const double A1 = Mzz*Mz + 4.0*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    const double A0 = Mzz*Cov_xy + (Mxz*Mxz*Myy + Myz*Myz*Mxx - 2.0*Mxz*Myz*Mxy) - Mz*(Mxz*Mxz + Myz*Myz);

    double x = 0.0;
    for (int it=0; it<25; ++it){
      const double f  = ((A3*x + A2)*x + A1)*x + A0;
      const double df = (3.0*A3*x + 2.0*A2)*x + A1;
      if (std::abs(df) < 1e-12) break;
      const double step = f/df;
      x -= step;
      if (std::abs(step) < 1e-12) break;
    }

    const double DET = x*x - x*Mz + Cov_xy;
    if (std::abs(DET) < 1e-12 || !std::isfinite(DET)) return false;

    const double uc = ( Mxz*(Myy - x) - Myz*Mxy ) / (2.0*DET);
    const double vc = ( Myz*(Mxx - x) - Mxz*Mxy ) / (2.0*DET);

    center.x = uc + meanx;
    center.y = vc + meany;
    R = std::sqrt(uc*uc + vc*vc + Mz + x);

    return std::isfinite(R) && R > 0.0;
  }

  // ---------- ★ ADDED: 헤비 코너 인식 ----------
  void detectCornerHeavy(const sensor_msgs::msg::LaserScan& s, double steer){
    if (!corner_enable_) { corner_active_heavy_ = false; corner_conf_=0.0; return; }

    // 1) 좌/우(안/바깥) 섹터 포인트 수집
    std::vector<Pt> in_pts, out_pts;
    collectSectorPoints(s, steer, +1.0, corner_side_deg_, ttc_corridor_half_, in_pts);
    collectSectorPoints(s, steer, -1.0, corner_side_deg_, ttc_corridor_half_, out_pts);

    if ((int)in_pts.size()  < std::max(6, min_inliers_/2) ||
        (int)out_pts.size() < std::max(6, min_inliers_/2)) {
      // 포인트 부족 → 신뢰도 하락
      updateCornerHysteresis(false, 0.0, steer);
      return;
    }

    // 2) 각 섹터 RANSAC 직선
    Line Lin, Lout; std::vector<int> in_idx, out_idx;
    bool ok_in  = ransacLine(in_pts,  ransac_iters_, ransac_tol_, Lin,  in_idx);
    bool ok_out = ransacLine(out_pts, ransac_iters_, ransac_tol_, Lout, out_idx);
    if (!ok_in || !ok_out || (int)in_idx.size() < min_inliers_ || (int)out_idx.size() < min_inliers_){
      updateCornerHysteresis(false, 0.0, steer);
      return;
    }

    // 3) 각도 + 교차점 확인
    double ang = lineAngleDeg(Lin, Lout);
    Pt cross;
    const bool interOK = lineIntersect(Lin, Lout, cross);
    const bool angleOK = (ang >= angle_thresh_deg_ && ang <= 160.0);
    const bool crossOK = (interOK && cross.x > 0.05 && cross.x <= intersect_max_dist_ &&
                          std::abs(cross.y) <= 2.0*ttc_corridor_half_);

    // 4) (옵션) 안쪽 inlier로 원 피팅 → 반경이 작을수록 코너 강화
    double Rfit = 1e9;
    Pt C{0,0};
    if (circle_use_){
      std::vector<Pt> inliers; inliers.reserve(in_idx.size());
      for (int idx: in_idx) inliers.push_back(in_pts[(size_t)idx]);
      double R;
      if (fitCirclePratt(inliers, C, R)) Rfit = R;
    }

    // 5) confidence 계산 → EMA
    double conf = 0.0;
    if (angleOK && crossOK) conf += 0.6;
    if (circle_use_ && Rfit < circle_R_max_) conf += 0.3;
    conf = std::clamp(conf, 0.0, 1.0);
    corner_conf_ = ema_alpha_*conf + (1.0-ema_alpha_)*corner_conf_;

    // 6) 히스테리시스 on/off
    const bool frame_corner = (corner_conf_ > 0.5);
    updateCornerHysteresis(frame_corner, corner_conf_, steer);

    // ---------- ★ ADDED: RViz 마커 출력 ----------
    if (debug_markers_) {
      visualization_msgs::msg::MarkerArray arr;
      const auto stamp = this->now();

      // (a) 교차점
      if (interOK) {
        visualization_msgs::msg::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = debug_frame_;
        m.ns = "pp_corner"; m.id = 1;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = cross.x;
        m.pose.position.y = cross.y;
        m.pose.position.z = 0.05;
        m.scale.x = m.scale.y = m.scale.z = 0.12;
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; // 주황
        arr.markers.push_back(m);
      }

      // (b) Pratt 원 (안쪽 벽 아크 근사)
      if (circle_use_ && Rfit < 1e8) {
        visualization_msgs::msg::Marker c;
        c.header.stamp = stamp;
        c.header.frame_id = debug_frame_;
        c.ns = "pp_corner"; c.id = 2;
        c.type = visualization_msgs::msg::Marker::LINE_STRIP;
        c.action = visualization_msgs::msg::Marker::ADD;
        c.scale.x = 0.02;
        c.color.a = 1.0; c.color.r = 0.0; c.color.g = 1.0; c.color.b = 0.0; // 초록
        geometry_msgs::msg::Point pt;
        const int N = 40;
        for (int i=0;i<=N;++i){
          double th = 2.0 * kPI() * (double)i / (double)N;
          pt.x = C.x + Rfit * std::cos(th);
          pt.y = C.y + Rfit * std::sin(th);
          pt.z = 0.01;
          c.points.push_back(pt);
        }
        arr.markers.push_back(c);
      }

      // (c) 상태 텍스트
      {
        visualization_msgs::msg::Marker t;
        t.header.stamp = stamp;
        t.header.frame_id = debug_frame_;
        t.ns = "pp_corner"; t.id = 3;
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.pose.position.x = 0.5; t.pose.position.y = 0.0; t.pose.position.z = 0.4;
        t.scale.z = 0.12;
        t.color.a = 1.0; t.color.r = 0.2; t.color.g = 0.8; t.color.b = 1.0;
        std::ostringstream oss;
        oss << "corner=" << (corner_active_heavy_?1:0)
            << " conf=" << std::fixed << std::setprecision(2) << corner_conf_;
        t.text = oss.str();
        arr.markers.push_back(t);
      }

      pub_markers_->publish(arr);
    }
  }

  void updateCornerHysteresis(bool corner_frame, double /*conf*/, double steer){
    if (corner_frame){
      corner_count_on_++; corner_count_off_ = 0;
      if (!corner_active_heavy_ && corner_count_on_ >= corner_on_cycles_){
        corner_active_heavy_ = true;
        RCLCPP_DEBUG(this->get_logger(), "HEAVY Corner ON (steer=%.3f)", steer);
        RCLCPP_INFO(this->get_logger(),   // ★ ADDED
                    "[CORNER] ON → TTC SUPPRESSED (steer=%.3f, conf=%.2f)",
                    steer, corner_conf_);
      }
    }else{
      corner_count_off_++; corner_count_on_ = 0;
      if (corner_active_heavy_ && corner_count_off_ >= corner_off_cycles_){
        corner_active_heavy_ = false;
        RCLCPP_DEBUG(this->get_logger(), "HEAVY Corner OFF");
        RCLCPP_INFO(this->get_logger(),   // ★ ADDED
                    "[CORNER] OFF → TTC ENABLED (conf=%.2f, last_ttc=%.2f)",
                    corner_conf_, ttc_min_last_);
      }
    }
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

    // ---------- ★ ADDED: 코너 인식(heavy) ----------
    if (has_ttc_scan_) {
      detectCornerHeavy(last_ttc_scan_, steer);
    } else {
      corner_active_heavy_ = false; corner_conf_=0.0;
    }

    // (1) 곡률 기반 속도 상한 (ay <= ay_max)  ← ★ curv_gain_speed_ 반영
    const double kappa_eff = std::max(1e-6, curv_gain_speed_ * std::abs(curvature));
    double v_curv = (kappa_eff < 1e-4) ? v_max_
               : std::sqrt(std::max(1e-6, ay_max_ / kappa_eff));
    v_curv = std::clamp(v_curv, v_min_, v_max_);

    // (2) TTC 기반 속도 상한 (게이트 + 필터 적용)
    double v_ttc = v_max_;
    double ttc_min = ttc_min_last_;                       // ★ ADDED: 기본값
    if (has_ttc_scan_) {
      // ★ CHANGED: 항상 계산해 기록(디버그용), 적용은 코너 아닐 때만
      ttc_min = computeMinTTC(
        last_ttc_scan_,
        std::max(0.0, v),
        ttc_fov_deg_ * kPI() / 180.0,   // deg→rad
        ttc_corridor_half_);
      ttc_min_last_ = ttc_min;          // ★ ADDED

      if (!corner_active_heavy_) {      // 코너 아닐 때만 TTC 개입
        if (ttc_min < ttc_gate_) {
          v_ttc = std::clamp(k_ttc_ * (ttc_min - ttc_safe_), 0.0, v_max_);
        } else {
          v_ttc = v_max_;
        }
      } else {
        v_ttc = v_max_;                 // 코너 Active → TTC 비활성
      }
    }

    // ----- ★ ADDED: 2초 주기 상태 요약(INFO) -----
    const bool ttc_suppressed = corner_active_heavy_;
    const bool ttc_capping = (!corner_active_heavy_ && has_ttc_scan_ && (ttc_min < ttc_gate_));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "[STATE] CORNER=%s | TTC=%s | ttc_min=%.2f gate=%.2f",
      (corner_active_heavy_ ? "ON" : "OFF"),
      (ttc_suppressed ? "OFF(by corner)" : (ttc_capping ? "CAP" : "IDLE")),
      ttc_min, ttc_gate_);

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

    // ---------- ★ ADDED: 디버그 상태 퍼블리시 ----------
    {
      std_msgs::msg::Bool b;      b.data   = corner_active_heavy_;
      std_msgs::msg::Float32 cf;  cf.data  = static_cast<float>(corner_conf_);
      std_msgs::msg::Float32 tt;  tt.data  = static_cast<float>(ttc_min_last_);
      pub_corner_active_->publish(b);
      pub_corner_conf_->publish(cf);
      pub_ttc_min_->publish(tt);
    }

    // 디버그(선택): 0.5s마다 수치 확인
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "v=%.2f v_ref=%.2f (curv=%.2f ttc=%.2f cap=%.2f) a=%.2f Ld=%.2f corner=%d conf=%.2f",
      v, v_ref, v_curv, ttc_min, v_ttc, a_cmd, Ld_target, (int)corner_active_heavy_, corner_conf_);
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

  // ---------- ★ ADDED: 코너 인식 상태/파라미터 ----------
  bool   corner_enable_{true};
  double corner_side_deg_{45.0};
  int    ransac_iters_{120};
  double ransac_tol_{0.05};
  int    min_inliers_{20};
  double angle_thresh_deg_{25.0};
  double intersect_max_dist_{3.0};
  bool   circle_use_{true};
  double circle_R_max_{6.0};
  double ema_alpha_{0.35};
  int    corner_on_cycles_{3};
  int    corner_off_cycles_{5};

  bool   corner_active_heavy_{false};
  double corner_conf_{0.0};
  int    corner_count_on_{0};
  int    corner_count_off_{0};

  // ---------- ★ ADDED: 디버그 퍼블리셔/플래그 ----------
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      pub_corner_active_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_corner_conf_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_ttc_min_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  bool        debug_markers_{true};
  std::string debug_frame_{"base_link"};
  double      ttc_min_last_{1e9};
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}

