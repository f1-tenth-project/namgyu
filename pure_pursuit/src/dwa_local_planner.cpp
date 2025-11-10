#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

using std::placeholders::_1;

/// ======================
/// Pure Pursuit (Path 추종)
/// ======================
class PurePursuit {
public:
    PurePursuit(double lookahead = 1.5, double speed = 2.0)
        : lookahead_distance_(lookahead), speed_(speed) {}

    void setPath(const nav_msgs::msg::Path::SharedPtr &path) {
        global_path_ = path;
    }

    ackermann_msgs::msg::AckermannDriveStamped follow(
    const nav_msgs::msg::Odometry::SharedPtr &odom)
{
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.drive.speed = speed_;

    if (!global_path_ || global_path_->poses.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("PurePursuit"), "경로 없음 → 정지");
        cmd.drive.speed = 0.0;
        cmd.drive.steering_angle = 0.0;
        return cmd;
    }

    double car_x = odom->pose.pose.position.x;
    double car_y = odom->pose.pose.position.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    double yaw = 2.0 * atan2(qz, qw);  // 단순 yaw

    // 1. 가장 가까운 waypoint 찾기
    double min_dist = 1e9;
    int closest_idx = 0;
    for (size_t i = 0; i < global_path_->poses.size(); i++) {
        double dx = global_path_->poses[i].pose.position.x - car_x;
        double dy = global_path_->poses[i].pose.position.y - car_y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 2. lookahead point 선택
    int target_idx = closest_idx;
    for (size_t i = closest_idx; i < global_path_->poses.size(); i++) {
        double dx = global_path_->poses[i].pose.position.x - car_x;
        double dy = global_path_->poses[i].pose.position.y - car_y;
        double dist = std::hypot(dx, dy);
        if (dist >= lookahead_distance_) {
            target_idx = i;
            break;
        }
    }

    double tx = global_path_->poses[target_idx].pose.position.x;
    double ty = global_path_->poses[target_idx].pose.position.y;

    // 3. 차량 좌표계로 변환
    double dx = tx - car_x;
    double dy = ty - car_y;
    double local_x = cos(-yaw) * dx - sin(-yaw) * dy;
    double local_y = sin(-yaw) * dx + cos(-yaw) * dy;

    // 4. Pure Pursuit 조향각 계산
    double L = 0.33; // wheelbase (F1Tenth 기본값)
    double ld = std::hypot(local_x, local_y);
    double alpha = atan2(local_y, local_x);
    double steering = atan2(2 * L * sin(alpha), ld);

    cmd.drive.steering_angle = steering;
    return cmd;
}


private:
    double lookahead_distance_;
    double speed_;
    nav_msgs::msg::Path::SharedPtr global_path_;
};

/// ======================
/// DWA (장애물 회피)
/// ======================
class DWA {
public:
    DWA(double max_speed = 1.5, double max_steering = 0.5)
        : max_speed_(max_speed), max_steering_(max_steering) {}

    ackermann_msgs::msg::AckermannDriveStamped compute(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        const nav_msgs::msg::Odometry::SharedPtr &odom)
    {
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.drive.speed = max_speed_;

        int mid = scan->ranges.size() / 2;
        double left_sum = 0, right_sum = 0;
        int count = 30;

        for (int i = 0; i < count; i++) {
            left_sum += scan->ranges[mid + i];
            right_sum += scan->ranges[mid - i];
        }

        if (left_sum > right_sum) {
            cmd.drive.steering_angle = -max_steering_;
        } else {
            cmd.drive.steering_angle = max_steering_;
        }

        return cmd;
    }

private:
    double max_speed_;
    double max_steering_;
};

/// ======================
/// Hybrid Planner 노드
/// ======================
class HybridPlanner : public rclcpp::Node {
public:
    HybridPlanner() : Node("hybrid_planner") {
        this->declare_parameter("obstacle_distance", 1.0);

        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd0", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan0", 10, std::bind(&HybridPlanner::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom0", 10, std::bind(&HybridPlanner::odom_callback, this, _1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/center_path", 10, std::bind(&HybridPlanner::path_callback, this, _1));

        pure_pursuit_ = std::make_shared<PurePursuit>();
        dwa_ = std::make_shared<DWA>();

        RCLCPP_INFO(this->get_logger(), "Hybrid Planner 시작 (/center_path + DWA)");
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    std::shared_ptr<PurePursuit> pure_pursuit_;
    std::shared_ptr<DWA> dwa_;

    double obstacle_distance_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        pure_pursuit_->setPath(msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        compute_and_publish();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
        compute_and_publish();
    }

    void compute_and_publish() {
        if (!latest_scan_ || !latest_odom_) return;

        bool obstacle = detect_obstacle(latest_scan_);
        ackermann_msgs::msg::AckermannDriveStamped cmd;

        if (obstacle) {
            RCLCPP_INFO(this->get_logger(), "⚠️ 장애물 발견 → DWA 실행");
            cmd = dwa_->compute(latest_scan_, latest_odom_);
        } else {
            cmd = pure_pursuit_->follow(latest_odom_);
        }

        cmd.header.stamp = this->get_clock()->now();
        cmd.header.frame_id = "base_link";
        drive_pub_->publish(cmd);
    }

    bool detect_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        for (auto r : scan->ranges) {
            if (std::isfinite(r) && r < obstacle_distance_) {
                return true;
            }
        }
        return false;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

