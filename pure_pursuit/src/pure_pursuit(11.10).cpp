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
#include <vector>
#include <limits>
#include <random>
#include <numeric>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>          // ★ 추가
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
    v_max_     = declare_parameter<double>("speed_max", 9.0);           // 직선 하드캡
    k_speed_   = declare_parameter<double>("k_speed",  2.5);            // (옵션)
    k_accel_   = declare_parameter<double>("k_accel",  4.0);            // 속도 P 이득
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
    ttc_scan_topic_= declare_parameter<std::string>("ttc_scan_topic", "scan0");

    tau_v_         = declare_parameter<double>("tau_v",    0.10);        // v_ref LPF

    // ----- v_ref 안정화(래이트 제한/데드밴드) -----
    a_ref_up_      = declare_parameter<double>("a_ref_up",   6.0);       // v_ref 상승 한계 [m/s^2]
    a_ref_down_    = declare_parameter<double>("a_ref_down", 3.5);       // v_ref 하강 한계 [m/s^2]
    v_deadband_    = declare_parameter<double>("v_deadband", 0.15);      // |v_ref - v| < dead → 0 처리

    // ★ TTC 필터 파라미터(정면 시야/복도폭)
    ttc_fov_deg_       = declare_parameter<double>("ttc_fov_deg", 20.0);     // 정면 각도창 ±deg
    ttc_corridor_half_ = declare_parameter<double>("ttc_corridor_half", 0.45); // 복도 반폭[m]

    // ★ 코너 완화 노브(곡률 가중치)
    curv_gain_speed_ = declare_parameter<double>("curv_gain_speed", 1.0);

    // ---------- 코너 인식 파라미터 ----------
    corner_enable_         = declare_parameter<bool>("corner_enable", true);
    corner_side_deg_       = declare_parameter<double>("corner_side_deg", .60);
    ransac_iters_          = declare_parameter<int>("corner_ransac_iters", 250);
    ransac_tol_            = declare_parameter<double>("corner_ransac_tol", 0.10);  // 완화
    min_inliers_           = declare_parameter<int>("corner_min_inliers", 8);      // 완화
    angle_thresh_deg_      = declare_parameter<double>("corner_angle_thresh_deg", 10.0);
    intersect_max_dist_    = declare_parameter<double>("corner_intersect_max", 10.0);
    circle_use_            = declare_parameter<bool>("corner_circle_use", false); // 현재 미사용
    circle_R_max_          = declare_parameter<double>("corner_circle_Rmax", 6.0);
    ema_alpha_             = declare_parameter<double>("corner_ema_alpha", 0.35);
    corner_on_cycles_      = declare_parameter<int>("corner_on_cycles", 3);
    corner_off_cycles_     = declare_parameter<int>("corner_off_cycles", 10);

    // 전역 탐색 보조 파라미터
    corner_y_max_          = declare_parameter<double>("corner_y_max", 3.5);
    corner_merge_radius_   = declare_parameter<double>("corner_merge_radius", 0.35);
    corner_max_lines_      = declare_parameter<int>("corner_max_lines", 8);

    // ★ 단일벽 폐색 에지 기반 코너 추정 파라미터
    occl_fov_deg_          = declare_parameter<double>("corner_occl_fov_deg", 80.0); // 정면 ±80°
    occl_jump_abs_         = declare_parameter<double>("corner_occl_jump_abs", 0.4); // Δr 절대 임계
    occl_jump_ratio_       = declare_parameter<double>("corner_occl_jump_ratio", 1.25); // r_{i+1}/r_i

    // 디버그
    debug_markers_ = declare_parameter<bool>("corner_debug_markers", true);
    debug_frame_   = declare_parameter<std::string>("corner_debug_frame", "base_link");

    // ★ 인코스 바이어스 파라미터 ------------------------------
    inner_bias_enable_       = declare_parameter<bool>("inner_bias_enable", true);
    inner_bias_kappa_thresh_ = declare_parameter<double>("inner_bias_kappa_thresh", 0.04);
    inner_bias_offset_max_   = declare_parameter<double>("inner_bias_offset_max", 0.35);
    inner_bias_gain_         = declare_parameter<double>("inner_bias_gain", 1.5);
    // --------------------------------------------------------

    // ★ 군집 기반 장애물 인식 파라미터 -------------------------
    obs_enable_           = declare_parameter<bool>("obs_enable", true);
    obs_fov_deg_          = declare_parameter<double>("obs_fov_deg", 60.0);   // 정면 ±60도
    obs_corridor_half_    = declare_parameter<double>("obs_corridor_half", 0.6); // 사용 복도 반폭
    obs_x_min_            = declare_parameter<double>("obs_x_min", 0.3);      // 차량 앞 최소 x
    obs_max_range_        = declare_parameter<double>("obs_max_range", 8.0);  // 최대 사용 거리
    obs_cluster_dist_     = declare_parameter<double>("obs_cluster_dist", 0.35); // 클러스터 거리 기준
    obs_min_points_       = declare_parameter<int>("obs_min_points", 3);         // 최소 포인트 수
    obs_front_y_thresh_   = declare_parameter<double>("obs_front_y_thresh", 0.35); // 정면 장애물 y 범위
    // --------------------------------------------------------

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

    pub_corner_active_ = create_publisher<std_msgs::msg::Bool>("/pp_debug/corner_active", 10);
    pub_corner_conf_   = create_publisher<std_msgs::msg::Float32>("/pp_debug/corner_conf", 10);
    pub_ttc_min_       = create_publisher<std_msgs::msg::Float32>("/pp_debug/ttc_min", 10);
    pub_markers_       = create_publisher<visualization_msgs::msg::MarkerArray>("/pp_debug/markers", 10);

    // 장애물 인식 디버그
    pub_obs_front_   = create_publisher<std_msgs::msg::Bool>("/pp_debug/obs_front", 10);
    pub_obs_dist_    = create_publisher<std_msgs::msg::Float32>("/pp_debug/obs_dist", 10);
    pub_obs_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/pp_debug/obs_markers", 10);

    // ★ 단일 장애물 마커 퍼블리셔 (정면 장애물 하나 표시용)
    pub_obs_single_marker_ =
        create_publisher<visualization_msgs::msg::Marker>("/pp_debug/obs_front_marker", 10);

    // 메인 제어 타이머
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&PurePursuitNode::onTimer, this));

    // ★ 장애물 마커 업데이트용 별도 타이머 (기존 함수 수정 X)
    obs_marker_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PurePursuitNode::onObsMarkerTimer, this));
  }

private:
  // ---------- 상수/유틸 ----------
  static constexpr double kPI() { return 3.14159265358979323846; }
  static inline double dot(double ax,double ay,double bx,double by){ return ax*bx+ay*by; }

  struct Pt { double x,y; };
  struct Line { double a,b,c; }; // ax+by+c=0 (정규화)

  struct RansacLine {
    Line L;
    std::vector<Pt> inliers;
  };
  struct Corner {
    Pt p;
    double ang_deg;
    double conf;
    int i, j;   // 라인 인덱스
  };

  // 장애물 클러스터
  struct ObstacleCluster {
    std::vector<Pt> pts;
    double cx{0.0};
    double cy{0.0};
    double r_min{1e9};
  };

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

  static double computeMinTTC(const sensor_msgs::msg::LaserScan& s,
                              double v, double fov_rad, double corridor_half)
  {
    if (v < 0.05) return 1e9;
    double ang = s.angle_min, min_ttc = 1e9;
    for (size_t i=0;i<s.ranges.size(); ++i, ang += s.angle_increment){
      const float r = s.ranges[i];
      if (!std::isfinite(r)) continue;
      if (std::abs(ang) > fov_rad) continue;
      const double y = r * std::sin(ang);
      if (std::abs(y) > corridor_half) continue;
      const double v_rel = v * std::cos(ang);
      if (v_rel <= 0) continue;
      const double ttc = r / std::max(1e-3, v_rel);
      if (ttc < min_ttc) min_ttc = ttc;
    }
    return min_ttc;
  }

  // ROI 완화: 더 멀리/넓게 보도록
  void collectGlobalPoints(const sensor_msgs::msg::LaserScan& s,
                           std::vector<Pt>& pts) const
  {
    pts.clear();
    pts.reserve(s.ranges.size());
    const double xmax = std::max(intersect_max_dist_ * 1.5, intersect_max_dist_);
    double ang = s.angle_min;
    for (size_t i=0;i<s.ranges.size(); ++i, ang += s.angle_increment){
      const float r = s.ranges[i];
      if (!std::isfinite(r) || r <= 0.05) continue;
      Pt p{ r*std::cos(ang), r*std::sin(ang) };
      if (p.x < 0.05 || p.x > xmax) continue;
      if (std::abs(p.y) > std::max(corner_y_max_, 2.5)) continue;
      pts.push_back(p);
    }
  }

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

  // --------- ★ 단일벽 상황: 폐색(occlusion) 에지 탐지 ---------
  bool findOcclusionEdge(const sensor_msgs::msg::LaserScan& s,
                         double fov_rad, double corridor_half,
                         double jump_abs, double jump_ratio,
                         Pt& p_occ, double& th_occ, double& score_out) const
  {
    double ang = s.angle_min, prev_ang = std::numeric_limits<double>::quiet_NaN();
    float prev_r = std::numeric_limits<float>::quiet_NaN();
    double best_score = -1.0;
    Pt best_pt{0,0}; double best_th = 0.0;

    for (size_t i=0;i<s.ranges.size(); ++i, ang += s.angle_increment){
      const float r = s.ranges[i];
      if (!std::isfinite(r) || r <= 0.05) continue;
      if (std::abs(ang) > fov_rad) continue;
      const double y = r * std::sin(ang);
      if (std::abs(y) > corridor_half) continue;

      if (std::isfinite(prev_r)) {
        const double dr = (double)r - (double)prev_r;
        const double ratio = (double)r / std::max(1e-6, (double)prev_r);
        // 벽이 끝나며 "갑자기 멀어지는" 지점(Δr > 0, ratio 큼)
        if (dr > jump_abs && ratio > jump_ratio) {
          const double x_prev = prev_r * std::cos(prev_ang);
          const double y_prev = prev_r * std::sin(prev_ang);
          const double score  = dr * ratio;
          if (score > best_score) {
            best_score = score;
            best_pt = {x_prev, y_prev};
            best_th = prev_ang;
          }
        }
      }
      prev_r = r; prev_ang = ang;
    }

    if (best_score > 0.0) {
      p_occ = best_pt; th_occ = best_th; score_out = best_score;
      return true;
    }
    return false;
  }

  // ---------- 코너 인식 (모든 코너, 단일벽 폴백 포함) ----------
  void detectCornerHeavy(const sensor_msgs::msg::LaserScan& s, double steer){
    corner_conf_ = 0.0;

    if (!corner_enable_) {
      corner_active_heavy_ = false;
      publishCornerMarkers(std::vector<Corner>{}, std::vector<RansacLine>{}, sensor_msgs::msg::LaserScan{});
      return;
    }

    // 1) 전역 포인트 수집
    std::vector<Pt> pool;
    collectGlobalPoints(s, pool);
    if ((int)pool.size() < std::max(6, min_inliers_)) {
      updateCornerHysteresis(false, 0.0, steer);
      publishCornerMarkers(std::vector<Corner>{}, std::vector<RansacLine>{}, sensor_msgs::msg::LaserScan{});
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[CORNER] pool=%zu → TOO FEW POINTS (min_inliers=%d)", pool.size(), min_inliers_);
      return;
    }

    // 2) 좌/우 분할 먼저 시도
    std::vector<Pt> pos, neg; pos.reserve(pool.size()); neg.reserve(pool.size());
    for (auto& p: pool) { (p.y >= 0 ? pos : neg).push_back(p); }

    auto fit_one = [&](const std::vector<Pt>& src, RansacLine& out)->bool{
      if ((int)src.size() < std::max(6, min_inliers_)) return false;
      Line L; std::vector<int> idx;
      if (!ransacLine(src, ransac_iters_, ransac_tol_, L, idx)) return false;
      if ((int)idx.size() < std::max(8, min_inliers_)) return false;
      out.L = L; out.inliers.clear(); out.inliers.reserve(idx.size());
      for (int i: idx) out.inliers.push_back(src[(size_t)i]);
      return true;
    };

    std::vector<RansacLine> lines;
    RansacLine Lpos, Lneg;
    bool okL = fit_one(pos, Lpos);
    bool okR = fit_one(neg, Lneg);
    if (okL) lines.push_back(Lpos);
    if (okR) lines.push_back(Lneg);

    // 3) 그래도 2개가 안 되면 글로벌 RANSAC로 보강
    if (lines.size() < 2) {
      std::vector<Pt> cur;
      cur.reserve(pool.size());
      auto used_by = [&](const Pt& p, const RansacLine& rl)->bool{
        return (lineDist(rl.L, p) < ransac_tol_);
      };
      for (auto& p : pool){
        bool used = false;
        for (auto& Lx: lines) { if (used_by(p,Lx)) { used=true; break; } }
        if (!used) cur.push_back(p);
      }

      for (int li=(int)lines.size(); li<corner_max_lines_; ++li) {
        if ((int)cur.size() < std::max(6, min_inliers_)) break;
        Line Lbest; std::vector<int> idx;
        if (!ransacLine(cur, ransac_iters_, ransac_tol_, Lbest, idx)) break;
        if ((int)idx.size() < std::max(8, min_inliers_)) break;

        RansacLine RL; RL.L = Lbest; RL.inliers.reserve(idx.size());
        std::vector<char> mark(cur.size(), 0);
        for (int id : idx) { mark[id] = 1; RL.inliers.push_back(cur[(size_t)id]); }
        std::vector<Pt> nxt; nxt.reserve(cur.size() - RL.inliers.size());
        for (size_t k=0;k<cur.size();++k) if (!mark[k]) nxt.push_back(cur[k]);
        cur.swap(nxt);
        lines.push_back(std::move(RL));
        if ((int)lines.size() >= 2) break;
      }
    }

    // 4) 라인이 1개뿐이면 — 단일벽 폐색 에지 폴백
    std::vector<Corner> corners;
    if (lines.size() == 1) {
      Pt occ; double th_occ=0.0, score=0.0;
      const double fov_rad = occl_fov_deg_ * kPI()/180.0;
      if (findOcclusionEdge(s, fov_rad, std::max(corner_y_max_, ttc_corridor_half_),
                            occl_jump_abs_, occl_jump_ratio_, occ, th_occ, score))
      {
        const double n_score = std::min(1.0,
          (double)lines[0].inliers.size() / (double)std::max(10, min_inliers_));
        const double j_score = std::min(1.0, score / (occl_jump_abs_*occl_jump_ratio_*2.0));
        const double conf = 0.5*j_score + 0.5*n_score;

        corners.push_back(Corner{occ, 90.0, conf, 0, -1});

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[CORNER-1WALL] occ=(%.2f, %.2f) score=%.2f n_in=%zu conf=%.2f",
          occ.x, occ.y, score, lines[0].inliers.size(), conf);
      }
    }

    // 5) 라인이 2개 이상이면 — 일반 교점 기반 코너
    if (corners.empty() && lines.size() >= 2) {
      auto add_or_merge = [&](const Corner& c){
        for (auto& e : corners){
          const double d = std::hypot(c.p.x-e.p.x, c.p.y-e.p.y);
          if (d <= corner_merge_radius_){ if (c.conf > e.conf) e = c; return; }
        }
        corners.push_back(c);
      };
      for (size_t i=0;i<lines.size();++i){
        for (size_t j=i+1;j<lines.size();++j){
          double ang = lineAngleDeg(lines[i].L, lines[j].L);
          if (!(ang >= angle_thresh_deg_ && ang <= 170.0)) continue;
          Pt cross;
          if (!lineIntersect(lines[i].L, lines[j].L, cross)) continue;
          if (!(cross.x > 0.05 && cross.x <= intersect_max_dist_)) continue;
          if (std::abs(cross.y) > corner_y_max_) continue;

          const double a_score = std::min(1.0, std::abs(ang)/90.0);
          const double n1 = (double)lines[i].inliers.size();
          const double n2 = (double)lines[j].inliers.size();
          const double n_score = std::min(1.0, std::min(n1,n2)/(double)std::max(10, min_inliers_));
          const double conf = 0.6*a_score + 0.4*n_score;
          add_or_merge(Corner{cross, ang, conf, (int)i, (int)j});
        }
      }
    }

    // 6) 조향방향 윈도우 내 코너 존재 여부 → 히스테리시스
    bool any_front = false;
    double best_conf = 0.0;
    const double sideWin = corner_side_deg_ * kPI() / 180.0;
    for (const auto& c : corners){
      const double th  = std::atan2(c.p.y, c.p.x);
      const double dth = std::atan2(std::sin(th - steer), std::cos(th - steer));
      if (std::abs(dth) <= sideWin && c.p.x > 0.05){ any_front = true; best_conf = std::max(best_conf, c.conf); }
    }
    corner_conf_ = ema_alpha_*best_conf + (1.0-ema_alpha_)*corner_conf_;
    updateCornerHysteresis(any_front && (corner_conf_>0.5), corner_conf_, steer);

    // 7) 마커/로그
    publishCornerMarkers(corners, lines, s);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "[CORNER] pool=%zu | lines=%zu | corners=%zu | active=%d conf=%.2f",
      pool.size(), lines.size(), corners.size(),
      (int)corner_active_heavy_, corner_conf_);
  }

  void updateCornerHysteresis(bool corner_frame, double /*conf*/, double steer){
    if (corner_frame){
      corner_count_on_++; corner_count_off_ = 0;
      if (!corner_active_heavy_ && corner_count_on_ >= corner_on_cycles_){
        corner_active_heavy_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "[CORNER] ON → TTC SUPPRESSED (steer=%.3f, conf=%.2f)",
                    steer, corner_conf_);
      }
    }else{
      corner_count_off_++; corner_count_on_ = 0;
      if (corner_active_heavy_ && corner_count_off_ >= corner_off_cycles_){
        corner_active_heavy_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "[CORNER] OFF → TTC ENABLED (conf=%.2f, last_ttc=%.2f)",
                    corner_conf_, ttc_min_last_);
      }
    }
  }

  void publishCornerMarkers(const std::vector<Corner>& corners,
                            const std::vector<RansacLine>& lines,
                            const sensor_msgs::msg::LaserScan& /*s*/)
  {
    if (!debug_markers_) return;
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = this->now();

    int id = 100;

    // (A) 코너: SPHERE + TEXT
    for (size_t k=0;k<corners.size();++k){
      const auto& c = corners[k];
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp; m.header.frame_id = debug_frame_;
      m.ns = "pp_corner_all"; m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.p.x; m.pose.position.y = c.p.y; m.pose.position.z = 0.05;
      m.scale.x = m.scale.y = m.scale.z = 0.14;
      m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.8; m.color.b = 0.0;
      arr.markers.push_back(m);

      visualization_msgs::msg::Marker t;
      t.header.stamp = stamp; t.header.frame_id = debug_frame_;
      t.ns = "pp_corner_all"; t.id = id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = c.p.x; t.pose.position.y = c.p.y; t.pose.position.z = 0.28;
      t.scale.z = 0.12;
      t.color.a = 1.0; t.color.r = 0.2; t.color.g = 0.9; t.color.b = 1.0;
      std::ostringstream oss;
      oss << "ang=" << std::fixed << std::setprecision(0) << c.ang_deg
          << " conf=" << std::fixed << std::setprecision(2) << c.conf;
      t.text = oss.str();
      arr.markers.push_back(t);
    }

    // (B) 라인: LINE_STRIP
    auto line_to_strip = [&](const Line& L, int id_local, float r,float g,float b){
      visualization_msgs::msg::Marker ls;
      ls.header.stamp = stamp; ls.header.frame_id = debug_frame_;
      ls.ns = "pp_corner_all"; ls.id = id_local;
      ls.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ls.action = visualization_msgs::msg::Marker::ADD;
      ls.scale.x = 0.02;
      ls.color.a = 1.0; ls.color.r = r; ls.color.g = g; ls.color.b = b;

      const double xmax = std::max(0.5, intersect_max_dist_);
      geometry_msgs::msg::Point gp;
      if (std::abs(L.b) > 1e-6) {
        for (double x=0.05; x<=xmax; x+=0.05){
          double y = (-L.a*x - L.c)/L.b;
          if (std::abs(y) > corner_y_max_) continue;
          gp.x=x; gp.y=y; gp.z=0.01; ls.points.push_back(gp);
        }
      } else if (std::abs(L.a) > 1e-6) {
        const double x = -L.c / L.a;
        if (x >= 0.05 && x <= xmax){
          for (double y=-corner_y_max_; y<=corner_y_max_; y+=0.05){
            gp.x=x; gp.y=y; gp.z=0.01; ls.points.push_back(gp);
          }
        }
      }
      return ls;
    };

    for (size_t i=0;i<lines.size();++i){
      float r = 0.2f + 0.6f * ((i%3)==0);
      float g = 0.2f + 0.6f * ((i%3)==1);
      float b = 0.2f + 0.6f * ((i%3)==2);
      arr.markers.push_back(line_to_strip(lines[i].L, id++, r,g,b));
    }

    // (C) 상태 텍스트
    {
      visualization_msgs::msg::Marker t;
      t.header.stamp = stamp; t.header.frame_id = debug_frame_;
      t.ns = "pp_corner_all"; t.id = id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = 0.6; t.pose.position.y = 0.0; t.pose.position.z = 0.45;
      t.scale.z = 0.13;
      t.color.a = 1.0; t.color.r = 0.9; t.color.g = 0.9; t.color.b = 0.9;
      std::ostringstream oss;
      oss << "corners=" << corners.size()
          << " conf=" << std::fixed << std::setprecision(2) << corner_conf_
          << " cornerActive=" << (corner_active_heavy_?1:0);
      t.text = oss.str();
      arr.markers.push_back(t);
    }

    pub_markers_->publish(arr);
  }

  // 1차 LPF
  static double lpf(double x, double x_prev, double tau, double dt){
    const double a = dt/(tau+dt);
    return x_prev + a*(x - x_prev);
  }

  // 비대칭 Slew
  static double slew(double target, double prev, double up, double down, double dt){
    const double dv = target - prev;
    const double max_step = (dv >= 0.0 ? up*dt : down*dt);
    if (std::abs(dv) > std::abs(max_step)) {
      return prev + std::copysign(std::abs(max_step), dv);
    }
    return target;
  }

  // ---------- 군집 기반 장애물 인식 -------------------------------
  void detectObstacleClusters(const sensor_msgs::msg::LaserScan& s)
  {
    obstacle_clusters_.clear();
    obstacle_front_     = false;
    obstacle_front_dist_= 1e9;
    obstacle_front_y_   = 0.0;

    if (!obs_enable_ || s.ranges.empty()) return;

    const double fov_rad   = obs_fov_deg_ * kPI() / 180.0;
    const double x_min     = obs_x_min_;
    const double y_half    = obs_corridor_half_;
    const double max_r     = obs_max_range_;
    const double th_dist   = obs_cluster_dist_;
    const int    min_pts   = std::max(1, obs_min_points_);

    std::vector<Pt> cur_pts;
    bool have_cluster = false;
    double prev_x = 0.0, prev_y = 0.0;

    double ang = s.angle_min;

    auto flush_cluster = [&](void){
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
      obstacle_clusters_.push_back(c);

      cur_pts.clear();
      have_cluster = false;
    };

    for (size_t i=0; i<s.ranges.size(); ++i, ang += s.angle_increment) {
      float r = s.ranges[i];

      bool valid = true;
      if (!std::isfinite(r) || r <= 0.05f) valid = false;
      if (valid && r > (float)max_r) r = (float)max_r;

      if (valid && std::abs(ang) > fov_rad) valid = false;

      double x=0.0, y=0.0;
      if (valid) {
        x = r * std::cos(ang);
        y = r * std::sin(ang);
        if (x < x_min || std::abs(y) > y_half) valid = false;
      }

      if (!valid) {
        // 유효하지 않은 점 → 클러스터 끊기
        if (have_cluster) {
          flush_cluster();
        }
        continue;
      }

      // 유효한 점
      Pt pt{x, y};
      if (!have_cluster) {
        cur_pts.clear();
        cur_pts.push_back(pt);
        prev_x = x; prev_y = y;
        have_cluster = true;
      } else {
        double d = std::hypot(x - prev_x, y - prev_y);
        if (d > th_dist) {
          // 새로운 클러스터 시작
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

    // 마지막 클러스터 처리
    flush_cluster();

    // 정면 장애물 판단: 중심 y가 작은 클러스터들 중 최소 거리
    for (const auto& c : obstacle_clusters_) {
      if (std::abs(c.cy) <= obs_front_y_thresh_) {
        if (c.r_min < obstacle_front_dist_) {
          obstacle_front_dist_ = c.r_min;
          obstacle_front_y_    = c.cy;
          obstacle_front_      = true;
        }
      }
    }

    // 디버그 마커 출력 (클러스터 전체)
    publishObstacleMarkers(obstacle_clusters_);
  }

  void publishObstacleMarkers(const std::vector<ObstacleCluster>& clusters)
  {
    if (!debug_markers_) return;

    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = this->now();
    int id = 0;

    // 각 클러스터 중심에 구(SPHERE)와 텍스트
    for (const auto& c : clusters) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp; m.header.frame_id = debug_frame_;
      m.ns = "pp_obstacles"; m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.cx;
      m.pose.position.y = c.cy;
      m.pose.position.z = 0.05;
      m.scale.x = m.scale.y = m.scale.z = 0.18;
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = 0.2;
      m.color.b = 0.2;
      arr.markers.push_back(m);

      visualization_msgs::msg::Marker t;
      t.header.stamp = stamp; t.header.frame_id = debug_frame_;
      t.ns = "pp_obstacles"; t.id = id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = c.cx;
      t.pose.position.y = c.cy;
      t.pose.position.z = 0.30;
      t.scale.z = 0.12;
      t.color.a = 1.0;
      t.color.r = 1.0;
      t.color.g = 1.0;
      t.color.b = 1.0;
      std::ostringstream oss;
      oss << "rmin=" << std::fixed << std::setprecision(2) << c.r_min;
      t.text = oss.str();
      arr.markers.push_back(t);
    }

    // 상태 텍스트 하나
    {
      visualization_msgs::msg::Marker t;
      t.header.stamp = stamp; t.header.frame_id = debug_frame_;
      t.ns = "pp_obstacles"; t.id = id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose.position.x = 0.6;
      t.pose.position.y = 0.0;
      t.pose.position.z = 0.55;
      t.scale.z = 0.13;
      t.color.a = 1.0;
      t.color.r = 0.8;
      t.color.g = 1.0;
      t.color.b = 0.8;
      std::ostringstream oss;
      oss << "obs_front=" << (obstacle_front_ ? 1 : 0)
          << " dist=" << std::fixed << std::setprecision(2) << obstacle_front_dist_;
      t.text = oss.str();
      arr.markers.push_back(t);
    }

    pub_obs_markers_->publish(arr);
  }
  // ----------------------------------------------------------------

  // ★ 정면 장애물 하나만 찍는 Marker (SPHERE) 퍼블리셔 -------------
  void publishObstacleMarker(bool has_obstacle, double x_obs, double y_obs)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = debug_frame_;      // 보통 "base_link"
    m.ns = "pp_obstacle_front";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = has_obstacle
               ? visualization_msgs::msg::Marker::ADD
               : visualization_msgs::msg::Marker::DELETE;

    m.pose.position.x = x_obs;
    m.pose.position.y = y_obs;
    m.pose.position.z = 0.1;               // 살짝 띄우기
    m.pose.orientation.w = 1.0;           // 회전 없음

    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;

    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;                      // 초록색
    m.color.b = 0.0;

    // lifetime은 기본값(0) → 계속 유지
    pub_obs_single_marker_->publish(m);
  }

  // ★ 별도 타이머 콜백: obstacle_front_ 상태를 보고 마커 갱신 -----
  void onObsMarkerTimer()
  {
    if (!debug_markers_) return;

    if (obstacle_front_) {
      // detectObstacleClusters에서 세팅된 가장 가까운 정면 장애물
      double x_obs = obstacle_front_dist_;
      double y_obs = obstacle_front_y_;
      publishObstacleMarker(true, x_obs, y_obs);
    } else {
      // 장애물 없으면 마커 삭제
      publishObstacleMarker(false, 0.0, 0.0);
    }
  }
  // ----------------------------------------------------------------

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

    // ★ 인코스 바이어스 적용을 위한 목표점 초기값 (센터라인)
    double goal_x = tp.x;
    double goal_y = tp.y;

    if (inner_bias_enable_) {
      const double kappa_path = computePathCurvature(center_path_, target_idx);
      const double kappa_abs  = std::fabs(kappa_path);

      if (kappa_abs > inner_bias_kappa_thresh_) {
        double d = inner_bias_gain_ * (kappa_abs - inner_bias_kappa_thresh_);
        if (d > inner_bias_offset_max_) d = inner_bias_offset_max_;
        if (d < 0.0) d = 0.0;

        const double yaw_path = computePathYaw(center_path_, target_idx);

        double nx = -std::sin(yaw_path);
        double ny =  std::cos(yaw_path);
        if (kappa_path < 0.0) { // 우코너 → 반대 방향
          nx = -nx;
          ny = -ny;
        }

        goal_x += nx * d;
        goal_y += ny * d;
      }
    }

    // 차량좌표계
    const double dx = goal_x - x;
    const double dy = goal_y - y;
    if (dx <= 0.01 && std::hypot(dx,dy) < 0.1) return;

    const double xL =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (xL <= 0.01) return;

    // PP 기하
    const double Ld = std::hypot(xL, yL);
    const double curvature = 2.0 * yL / (Ld * Ld);
    const double steer_pp  = std::atan(wheelbase_ * curvature);

    // 코너 인식
    if (has_ttc_scan_) {
      detectCornerHeavy(last_ttc_scan_, steer_pp);
    } else {
      corner_active_heavy_ = false; corner_conf_=0.0;
    }

    // (1) 곡률 기반 속도 상한
    const double kappa_eff = std::max(1e-6, curv_gain_speed_ * std::abs(curvature));
    double v_curv = (kappa_eff < 1e-4) ? v_max_
               : std::sqrt(std::max(1e-6, ay_max_ / kappa_eff));
    v_curv = std::clamp(v_curv, v_min_, v_max_);

    // (2) TTC 기반 속도 상한
    double v_ttc = v_max_;
    double ttc_min = ttc_min_last_;
    if (has_ttc_scan_) {
      ttc_min = computeMinTTC(
        last_ttc_scan_,
        std::max(0.0, v),
        ttc_fov_deg_ * kPI() / 180.0,
        ttc_corridor_half_);
      ttc_min_last_ = ttc_min;

      if (!corner_active_heavy_) {
        if (ttc_min < ttc_gate_) {
          v_ttc = std::clamp(k_ttc_ * (ttc_min - ttc_safe_), 0.0, v_max_);
        } else {
          v_ttc = v_max_;
        }
      } else {
        v_ttc = v_max_; // 코너 활성 시 TTC 억제
      }

      // ★ 여기서 군집 기반 장애물 인식 수행 (제어에는 아직 미반영, 디버그만)
      detectObstacleClusters(last_ttc_scan_);
      {
        std_msgs::msg::Bool b;      b.data   = obstacle_front_;
        std_msgs::msg::Float32 d;   d.data   = static_cast<float>(obstacle_front_dist_);
        pub_obs_front_->publish(b);
        pub_obs_dist_->publish(d);
      }

    } else {
      obstacle_front_      = false;
      obstacle_front_dist_ = 1e9;
    }

    // (3) 최종 v_ref: min(곡률, TTC) → LPF → Slew
    const double v_ref_raw = std::min(v_curv, v_ttc);
    const double v_ref_lpf = lpf(v_ref_raw, v_cmd_prev_, tau_v_, 0.01);
    const double v_ref     = slew(v_ref_lpf, v_cmd_prev_, a_ref_up_, a_ref_down_, 0.01);
    v_cmd_prev_ = v_ref;

    // (4) 데드밴드 + P제어 + 포화
    double err = v_ref - v;
    if (std::abs(err) < v_deadband_) err = 0.0;

    double a_cmd = k_accel_ * err;
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;

    // 현재는 steer_pp 그대로 사용 (장애물 회피 스티어링은 아직 X)
    double steer_cmd = steer_pp;

    // 퍼블리시
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link0";
    cmd.drive.steering_angle = steer_cmd;
    cmd.drive.acceleration   = a_cmd;
    cmd.drive.speed          = 0.0;

    drive_pub_->publish(cmd);

    // 디버그 퍼블리시
    {
      std_msgs::msg::Bool b;      b.data   = corner_active_heavy_;
      std_msgs::msg::Float32 cf;  cf.data  = static_cast<float>(corner_conf_);
      std_msgs::msg::Float32 tt;  tt.data  = static_cast<float>(ttc_min_last_);
      pub_corner_active_->publish(b);
      pub_corner_conf_->publish(cf);
      pub_ttc_min_->publish(tt);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "v=%.2f v_ref=%.2f (curv=%.2f ttc=%.2f cap=%.2f) a=%.2f Ld=%.2f steer=%.3f obs_front=%d dist=%.2f",
      v, v_ref, v_curv, ttc_min, v_ttc, a_cmd, Ld_target,
      steer_cmd, (int)obstacle_front_, obstacle_front_dist_);
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

  // 인코스용: center_path의 국소 곡률 계산
  static double computePathCurvature(const nav_msgs::msg::Path &path,
                                     int idx)
  {
    const auto &poses = path.poses;
    const int n = (int)poses.size();
    if (n < 3) return 0.0;

    int i = std::clamp(idx, 1, n-2);
    const auto &pm = poses[i-1].pose.position;
    const auto &p  = poses[i].pose.position;
    const auto &pp = poses[i+1].pose.position;

    const double yaw1 = std::atan2(p.y - pm.y,  p.x - pm.x);
    const double yaw2 = std::atan2(pp.y - p.y,  pp.x - p.x);
    const double dyaw = std::atan2(std::sin(yaw2 - yaw1), std::cos(yaw2 - yaw1));
    const double ds   = std::hypot(pp.x - pm.x, pp.y - pm.y);
    if (ds < 1e-3) return 0.0;

    return dyaw / ds;   // [1/m]
  }

  // 인코스용: center_path의 국소 진행방향 yaw 계산
  static double computePathYaw(const nav_msgs::msg::Path &path,
                               int idx)
  {
    const auto &poses = path.poses;
    const int n = (int)poses.size();
    if (n < 2) return 0.0;

    int i0 = std::max(0, idx-1);
    int i1 = std::min(n-1, idx+1);
    const auto &p0 = poses[i0].pose.position;
    const auto &p1 = poses[i1].pose.position;

    return std::atan2(p1.y - p0.y, p1.x - p0.x);
  }

  // ----- 멤버 -----
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_center_, sub_left_, sub_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ttc_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr obs_marker_timer_;  // ★ 추가: 장애물 마커용 타이머

  nav_msgs::msg::Path center_path_, left_path_, right_path_;
  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::LaserScan last_ttc_scan_;
  bool has_odom_{false}, has_ttc_scan_{false};

  // 파라미터/상태
  double lookahead_, wheelbase_;
  double v_min_, v_max_, k_speed_;
  double k_accel_, a_min_, a_max_;
  std::string center_path_topic_, left_path_topic_, right_path_topic_, odom_topic_, drive_topic_;

  double ay_max_, ld0_, kv_ld_, ld_min_, ld_max_;
  double ttc_safe_, k_ttc_, tau_v_, ttc_gate_;
  std::string ttc_scan_topic_;

  double a_ref_up_, a_ref_down_, v_deadband_;
  double v_cmd_prev_{0.0};

  double ttc_fov_deg_, ttc_corridor_half_;
  double curv_gain_speed_;

  // 인코스 바이어스
  bool   inner_bias_enable_{false};
  double inner_bias_kappa_thresh_{0.04};
  double inner_bias_offset_max_{0.35};
  double inner_bias_gain_{1.5};

  // 코너 인식
  bool   corner_enable_{true};
  double corner_side_deg_{45.0};
  int    ransac_iters_{250};
  double ransac_tol_{0.08};
  int    min_inliers_{10};
  double angle_thresh_deg_{12.0};
  double intersect_max_dist_{10.0};
  bool   circle_use_{false};
  double circle_R_max_{6.0};
  double ema_alpha_{0.35};
  int    corner_on_cycles_{3};
  int    corner_off_cycles_{5};

  double corner_y_max_{3.5};
  double corner_merge_radius_{0.35};
  int    corner_max_lines_{8};

  // 단일벽 폐색 파라미터
  double occl_fov_deg_{60.0};
  double occl_jump_abs_{0.6};
  double occl_jump_ratio_{1.4};

  bool   corner_active_heavy_{false};
  double corner_conf_{0.0};
  int    corner_count_on_{0};
  int    corner_count_off_{0};

  // 디버그 (코너)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      pub_corner_active_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_corner_conf_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_ttc_min_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  bool        debug_markers_{true};
  std::string debug_frame_{"base_link0"};
  double      ttc_min_last_{1e9};

  // 장애물 인식 파라미터
  bool   obs_enable_{true};
  double obs_fov_deg_{60.0};
  double obs_corridor_half_{0.6};
  double obs_x_min_{0.3};
  double obs_max_range_{8.0};
  double obs_cluster_dist_{0.35};
  int    obs_min_points_{3};
  double obs_front_y_thresh_{0.35};

  // 군집 결과
  std::vector<ObstacleCluster> obstacle_clusters_;
  bool   obstacle_front_{false};
  double obstacle_front_dist_{1e9};
  double obstacle_front_y_{0.0};

  // 장애물 디버그 퍼블리셔
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      pub_obs_front_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_obs_dist_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obs_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      pub_obs_single_marker_; // ★ 단일 마커
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
