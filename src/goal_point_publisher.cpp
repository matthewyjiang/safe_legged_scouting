#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cmath>

class GoalPointPublisher : public rclcpp::Node {
public:
  GoalPointPublisher() : Node("goal_point_publisher") {
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "goal_point", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "goal_marker", 10);

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "goal_markers", 10);

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "spirit/current_pose", 10,
        std::bind(&GoalPointPublisher::pose_callback, this, std::placeholders::_1));


    goal_input_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "input_goal_point", 10,
        std::bind(&GoalPointPublisher::goal_input_callback, this, std::placeholders::_1));


    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&GoalPointPublisher::check_waypoint_progress, this));

    marker_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GoalPointPublisher::publish_all_goal_markers, this));


    increment_service_ = this->create_service<std_srvs::srv::Trigger>(
        "increment_goal_index",
        std::bind(&GoalPointPublisher::increment_goal_index, this, std::placeholders::_1, std::placeholders::_2));


    load_waypoints_from_config();
    current_waypoint_index_ = 0;
    
    publish_all_goal_markers();

    RCLCPP_INFO(this->get_logger(), "Goal Point Publisher initialized with %zu waypoints", waypoints_.size());
  }

private:
  void load_waypoints_from_config() {
    this->declare_parameter("waypoints", std::vector<double>{});
    auto waypoint_params = this->get_parameter("waypoints").as_double_array();
    
    if (waypoint_params.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Waypoints parameter must contain triplets of x,y,z coordinates");
      return;
    }
    
    for (size_t i = 0; i < waypoint_params.size(); i += 3) {
      geometry_msgs::msg::Point point;
      point.x = waypoint_params[i];
      point.y = waypoint_params[i + 1];
      point.z = waypoint_params[i + 2];
      waypoints_.push_back(point);
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from config", waypoints_.size());
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
    pose_received_ = true;
  }


  void goal_input_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    waypoints_.push_back(msg->point);
    RCLCPP_INFO(this->get_logger(), "Added new goal point: (%.2f, %.2f, %.2f). Total waypoints: %zu",
                msg->point.x, msg->point.y, msg->point.z, waypoints_.size());
  }

  void check_waypoint_progress() {
    if (waypoints_.empty() || current_waypoint_index_ >= waypoints_.size()) {
      return;
    }

    // Always publish current goal
    publish_current_goal();

    // Check if we have pose data to determine progress
    if (!pose_received_) {
      return;
    }

    double distance = calculate_distance(current_pose_.position, waypoints_[current_waypoint_index_]);
    
    RCLCPP_INFO(this->get_logger(), "Distance to waypoint %zu: %.3f m (robot: %.2f, %.2f) (goal: %.2f, %.2f)", 
                current_waypoint_index_, distance,
                current_pose_.position.x, current_pose_.position.y,
                waypoints_[current_waypoint_index_].x, waypoints_[current_waypoint_index_].y);
    
    if (distance <= 0.1) {
      current_waypoint_index_++;
      
      if (current_waypoint_index_ < waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint, moving to next waypoint %zu", current_waypoint_index_);
      } else {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached!");
      }
    }
  }

  void publish_current_goal() {
    if (current_waypoint_index_ >= waypoints_.size()) {
      return;
    }

    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();
    msg.point = waypoints_[current_waypoint_index_];

    publisher_->publish(msg);
    publish_goal_marker(msg.point);

    RCLCPP_INFO(this->get_logger(), "Published goal point %zu: (%.2f, %.2f, %.2f)",
                current_waypoint_index_, msg.point.x, msg.point.y, msg.point.z);
  }

  double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
  }

  void publish_goal_marker(const geometry_msgs::msg::Point &goal) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal.x;
    marker.pose.position.y = goal.y;
    marker.pose.position.z = goal.z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub_->publish(marker);
  }

  void increment_goal_index(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (waypoints_.empty()) {
      response->success = false;
      response->message = "No waypoints loaded";
      return;
    }

    if (current_waypoint_index_ >= waypoints_.size() - 1) {
      response->success = false;
      response->message = "Already at final waypoint";
      return;
    }

    current_waypoint_index_++;
    response->success = true;
    response->message = "Goal index incremented to " + std::to_string(current_waypoint_index_);
    
    RCLCPP_INFO(this->get_logger(), "Manually incremented to waypoint %zu", current_waypoint_index_);
  }


  void publish_all_goal_markers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "all_goals";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = waypoints_[i].x;
      marker.pose.position.y = waypoints_[i].y;
      marker.pose.position.z = waypoints_[i].z;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;

      if (i < current_waypoint_index_) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;
      } else if (i == current_waypoint_index_) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      } else {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
      }

      marker_array.markers.push_back(marker);
    }

    marker_array_pub_->publish(marker_array);
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_input_subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr increment_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  
  std::vector<geometry_msgs::msg::Point> waypoints_;
  size_t current_waypoint_index_;
  geometry_msgs::msg::Pose current_pose_;
  bool pose_received_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPointPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
