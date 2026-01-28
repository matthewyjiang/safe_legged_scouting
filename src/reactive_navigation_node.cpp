#include "polygeom_lib.h"
#include "reactive_planner_lib.h"
#include "safe_bayesian_optimization/msg/polygon_array.hpp"

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/detail/sections/sectionalize.hpp>
#include <boost/geometry/algorithms/line_interpolate.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace bg = boost::geometry;

class ReactiveNavigationNode : public rclcpp::Node {
public:
  ReactiveNavigationNode()
      : Node("reactive_navigation_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // Declare reactive planner parameters
    this->declare_parameter("p", 2.0);
    this->declare_parameter("epsilon", 20.0);
    this->declare_parameter("varepsilon", 0.05);
    this->declare_parameter("mu_1", 1.0);
    this->declare_parameter("mu_2", 0.15);
    this->declare_parameter("robot_radius", 0.3);
    this->declare_parameter("linear_gain", 1.0);
    this->declare_parameter("angular_gain", 1.0);
    this->declare_parameter("linear_cmd_limit", 0.5);
    this->declare_parameter("angular_cmd_limit", 1.0);
    this->declare_parameter("goal_tolerance", 0.1);
    this->declare_parameter("next_subgoal_tolerance", 0.1);

    // Read parameters and configure diffeomorphism parameters
    double p = this->get_parameter("p").as_double();
    double epsilon = this->get_parameter("epsilon").as_double();
    double varepsilon = this->get_parameter("varepsilon").as_double();
    double mu_1 = this->get_parameter("mu_1").as_double();
    double mu_2 = this->get_parameter("mu_2").as_double();
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    linear_cmd_limit_ = this->get_parameter("linear_cmd_limit").as_double();
    angular_cmd_limit_ = this->get_parameter("angular_cmd_limit").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    next_subgoal_tolerance_ =
        this->get_parameter("next_subgoal_tolerance").as_double();

    // Set default workspace (will be updated when envelope is received)
    std::vector<std::vector<double>> default_workspace = {
        {-10.0, -10.0}, {10.0, -10.0}, {10.0, 10.0}, {-10.0, 10.0}};

    // Configure diffeomorphism parameters
    diffeo_params_.set_all_params(p, epsilon, varepsilon, mu_1, mu_2,
                                  default_workspace);

    RCLCPP_INFO(
        this->get_logger(),
        "Configured reactive planner with p=%.2f, epsilon=%.3f, "
        "varepsilon=%.3f, mu_1=%.2f, mu_2=%.2f, robot_radius=%.3f, "
        "linear_gain=%.2f, angular_gain=%.2f, linear_limit=%.2f, "
        "angular_limit=%.2f, goal_tolerance=%.3f, next_subgoal_tol=%.3f",
        p, epsilon, varepsilon, mu_1, mu_2, robot_radius_, linear_gain_,
        angular_gain_, linear_cmd_limit_, angular_cmd_limit_, goal_tolerance_,
        next_subgoal_tolerance_);

    // Subscribe to obstacle polygons
    obstacles_sub_ = this->create_subscription<
        safe_bayesian_optimization::msg::PolygonArray>(
        "polygon_array", 10,
        std::bind(&ReactiveNavigationNode::obstacles_callback, this,
                  std::placeholders::_1));

    // Subscribe to envelope polygon
    envelope_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "envelope_polygon", 10,
        std::bind(&ReactiveNavigationNode::envelope_callback, this,
                  std::placeholders::_1));

    // Subscribe to pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "spirit/current_pose", 10,
        std::bind(&ReactiveNavigationNode::pose_callback, this,
                  std::placeholders::_1));

    // Subscribe to current subgoal
    subgoal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/current_subgoal", 10,
        std::bind(&ReactiveNavigationNode::subgoal_callback, this,
                  std::placeholders::_1));

    // Create control command publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "spirit/planner_twist_command", 10);

    // Publish a trigger when the current subgoal is reached
    subgoal_reached_pub_ =
        this->create_publisher<std_msgs::msg::Empty>("/subgoal_reached", 10);

    // Create freespace markers publisher
    freespace_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/freespace_markers", 10);
    local_workspace_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/local_workspace_markers", 10);

    // Create envelope markers publisher
    envelope_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/envelope_markers", 10);

    // Create obstacle centers markers publisher
    obstacle_centers_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacle_centers_markers", 10);

    // Create interpolated centers markers publisher
    interpolated_centers_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/interpolated_centers_markers", 10);

    // Create merged polygon markers publisher
    merged_polygon_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/merged_polygon_markers", 10);

    // Create clipped polygon markers publisher
    clipped_polygon_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/clipped_polygon_markers", 10);

    // Create local goal marker publisher
    local_goal_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/local_goal_marker", 10);

    // Create transformed position marker publisher
    transformed_position_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/transformed_position_marker", 10);

    // Create free space line marker publisher
    free_space_line_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/free_space_line_marker", 10);

    // Create timer for control loop
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ReactiveNavigationNode::control_callback, this));

    RCLCPP_INFO(this->get_logger(), "Reactive Navigation Node initialized");
  }

private:
  rclcpp::Subscription<safe_bayesian_optimization::msg::PolygonArray>::SharedPtr
      obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr envelope_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subgoal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr subgoal_reached_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      freespace_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      local_workspace_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      envelope_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      obstacle_centers_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      interpolated_centers_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      merged_polygon_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      clipped_polygon_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      local_goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      transformed_position_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      free_space_line_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  DiffeoParamsClass diffeo_params_;
  double robot_radius_;
  double linear_gain_;
  double angular_gain_;
  double linear_cmd_limit_;
  double angular_cmd_limit_;
  double goal_tolerance_;
  double next_subgoal_tolerance_;
  std::vector<polygon> obstacle_polygons_;
  std::vector<std::vector<PolygonClass>> diffeo_tree_array_;

  // Robot state
  geometry_msgs::msg::Point current_position_;
  double current_yaw_;
  bool has_robot_pose_ = false;

  // Subgoal state
  geometry_msgs::msg::Point current_subgoal_;
  bool has_subgoal_ = false;
  bool subgoal_reached_ = false;
  double env_x_min_ = 0.0;
  double env_x_max_ = 0.0;
  double env_y_min_ = 0.0;
  double env_y_max_ = 0.0;
  polygon envelope_polygon_map_;
  bool has_envelope_ = false;

  std::vector<polygon> get_merged_dilated_polygons() {
    if (obstacle_polygons_.empty()) {
      return {};
    }

    std::vector<polygon> polygon_list;
    polygon_list.reserve(obstacle_polygons_.size());

    // Dilate each polygon by robot radius using full buffer strategies
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(
        robot_radius_);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy;
    bg::strategy::buffer::side_straight side_strategy;

    for (const auto &poly : obstacle_polygons_) {
      multi_polygon dilated_result;
      boost::geometry::buffer(poly, dilated_result, distance_strategy,
                              side_strategy, join_strategy, end_strategy,
                              point_strategy);

      if (!dilated_result.empty()) {
        polygon_list.push_back(dilated_result.front());
      }
    }

    if (polygon_list.empty()) {
      return {};
    }

    // Merge all overlapping polygons using union operation
    multi_polygon output_union;
    if (polygon_list.size() >= 1) {
      output_union.push_back(polygon_list.back());
      polygon_list.pop_back();

      while (!polygon_list.empty()) {
        polygon next_polygon = polygon_list.back();
        polygon_list.pop_back();
        multi_polygon temp_result;
        bg::union_(output_union, next_polygon, temp_result);
        output_union = temp_result;
      }
    }

    // Simplify and create final merged polygon list
    std::vector<polygon> polygon_list_merged;
    for (size_t i = 0; i < output_union.size(); i++) {
      polygon simplified_component;
      bg::simplify(output_union[i], simplified_component, 0.2);
      polygon_list_merged.push_back(simplified_component);
    }

    return polygon_list_merged;
  }

  void obstacles_callback(
      const safe_bayesian_optimization::msg::PolygonArray::SharedPtr msg) {

    // Clear previous obstacles
    obstacle_polygons_.clear();
    obstacle_polygons_.reserve(msg->polygons.size());

    // Convert ROS polygons to boost geometry polygons
    for (const auto &ros_polygon : msg->polygons) {
      if (ros_polygon.points.size() < 3) {
        RCLCPP_WARN(this->get_logger(),
                    "Skipping polygon with less than 3 points");
        continue;
      }

      polygon boost_poly;

      // Add points to the polygon
      for (const auto &point : ros_polygon.points) {
        boost::geometry::append(
            boost_poly.outer(),
            bg::model::point<double, 2, bg::cs::cartesian>(
                static_cast<double>(point.x), static_cast<double>(point.y)));
      }

      // Close the polygon by adding the first point again if needed
      if (!boost::geometry::equals(boost_poly.outer().front(),
                                   boost_poly.outer().back())) {
        boost::geometry::append(boost_poly.outer(), boost_poly.outer().front());
      }

      // Correct the polygon (ensure proper orientation and closure)
      boost::geometry::correct(boost_poly);

      obstacle_polygons_.push_back(boost_poly);
    }

    auto merged_polygons = get_merged_dilated_polygons();

    // Publish merged polygon markers for visualization
    publish_merged_polygon_markers(merged_polygons);

    // Get envelope polygon for intersection
    std::vector<std::vector<double>> workspace = diffeo_params_.get_workspace();
    polygon envelope_polygon;
    if (!workspace.empty()) {
      for (const auto &vertex : workspace) {
        bg::append(envelope_polygon.outer(),
                   bg::model::point<double, 2, bg::cs::cartesian>(vertex[0],
                                                                  vertex[1]));
      }
      bg::correct(envelope_polygon);
    }

    // Collect clipped polygons for visualization
    std::vector<polygon> clipped_polygons;

    diffeo_tree_array_.clear();
    for (const auto &merged_poly : merged_polygons) {
      if (merged_poly.outer().size() < 3) {
        RCLCPP_WARN(this->get_logger(),
                    "Skipping merged polygon with less than 3 vertices");
        continue;
      }

      // Intersect merged polygon with envelope polygon
      multi_polygon intersection_result;
      polygon clipped_poly = merged_poly;

      if (!workspace.empty() && bg::area(envelope_polygon) > 0.001) {
        bg::intersection(merged_poly, envelope_polygon, intersection_result);

        if (intersection_result.empty()) {
          RCLCPP_WARN(
              this->get_logger(),
              "Merged polygon has no intersection with envelope, skipping");
          continue;
        }

        // Use the largest intersection component
        double max_area = 0.0;
        for (const auto &poly : intersection_result) {
          double area = bg::area(poly);
          if (area > max_area) {
            max_area = area;
            clipped_poly = poly;
          }
        }
      }

      // Skip if clipped polygon is too small
      if (bg::area(clipped_poly) < 0.01 || clipped_poly.outer().size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Clipped polygon too small, skipping");
        continue;
      }

      // Add valid clipped polygon to collection for visualization
      clipped_polygons.push_back(clipped_poly);

      std::vector<PolygonClass> tree;
      diffeoTreeConvex(BoostPointToStd(BoostPolyToBoostPoint(clipped_poly)),
                       diffeo_params_, &tree);
      diffeo_tree_array_.push_back(tree);
    }

    // Publish clipped polygon markers for visualization
    publish_clipped_polygon_markers(clipped_polygons);

    update_transformed_envelope();
  }

  void update_transformed_envelope() {
    if (!has_envelope_) {
      return;
    }

    polygon envelope_polygon_diffeo;
    const auto &outer = envelope_polygon_map_.outer();
    if (outer.size() < 3) {
      return;
    }

    const size_t max_index = outer.size() - 1; // skip duplicate close point
    for (size_t i = 0; i < max_index; ++i) {
      std::vector<double> position = {outer[i].get<0>(), outer[i].get<1>()};
      DiffeoTransformResult transform_result =
          computeDiffeoTransform(position, 0.0, diffeo_tree_array_,
                                 diffeo_params_, this->get_logger());
      bg::append(envelope_polygon_diffeo.outer(),
                 bg::model::point<double, 2, bg::cs::cartesian>(
                     transform_result.transformed_position[0],
                     transform_result.transformed_position[1]));
    }

    if (!envelope_polygon_diffeo.outer().empty()) {
      bg::append(envelope_polygon_diffeo.outer(),
                 envelope_polygon_diffeo.outer().front());
    }
    bg::correct(envelope_polygon_diffeo);

    bg::model::box<bg::model::d2::point_xy<double>> bounds;
    bg::envelope(envelope_polygon_diffeo, bounds);
    env_x_min_ = bounds.min_corner().get<0>();
    env_y_min_ = bounds.min_corner().get<1>();
    env_x_max_ = bounds.max_corner().get<0>();
    env_y_max_ = bounds.max_corner().get<1>();

    publish_envelope_markers(envelope_polygon_diffeo);
  }

  void envelope_callback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
    // Convert ROS polygon to workspace format
    std::vector<std::vector<double>> workspace;
    workspace.reserve(msg->points.size() + 1);

    const auto &first_point = msg->points.front();

    for (const auto &point : msg->points) {
      std::vector<double> vertex = {static_cast<double>(point.x),
                                    static_cast<double>(point.y)};
      workspace.push_back(vertex);
    }

    workspace.push_back({static_cast<double>(first_point.x),
                         static_cast<double>(first_point.y)});

    // Update workspace in diffeomorphism parameters
    diffeo_params_.set_workspace(workspace);

    // Store the envelope polygon in map frame for later diffeo transform
    envelope_polygon_map_.outer().clear();
    for (const auto &point : msg->points) {
      bg::append(
          envelope_polygon_map_.outer(),
          bg::model::point<double, 2, bg::cs::cartesian>(point.x, point.y));
    }
    if (!envelope_polygon_map_.outer().empty()) {
      bg::append(envelope_polygon_map_.outer(),
                 envelope_polygon_map_.outer().front());
    }
    bg::correct(envelope_polygon_map_);
    has_envelope_ = true;

    update_transformed_envelope();
  }

  void subgoal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    current_subgoal_ = *msg;
    has_subgoal_ = true;
    subgoal_reached_ = false;

    RCLCPP_INFO(this->get_logger(), "Received subgoal: x=%.3f, y=%.3f, z=%.3f",
                current_subgoal_.x, current_subgoal_.y, current_subgoal_.z);
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Update current position from pose
    current_position_ = msg->position;

    // Extract yaw from quaternion
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    has_robot_pose_ = true;
  }

  void control_callback() {
    if (!has_robot_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "No robot pose available for control");
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    std::vector<double> robot_position = {current_position_.x,
                                          current_position_.y};

    // print the size of the diffeo tree array
    RCLCPP_INFO(this->get_logger(), "Diffeo tree array size: %zu",
                diffeo_tree_array_.size());

    DiffeoTransformResult transform_result =
        computeDiffeoTransform(robot_position, current_yaw_, diffeo_tree_array_,
                               diffeo_params_, this->get_logger());

    // Publish transformed position marker for visualization
    publish_transformed_position_marker(transform_result.transformed_position);

    std::vector<point> model_obstacle_centers;
    std::vector<double> model_obstacle_radii;

    for (const auto &tree : diffeo_tree_array_) {
      if (tree.empty()) {
        RCLCPP_WARN(this->get_logger(), "Diffeo tree is empty, skipping");
        continue;
      }

      point root_center = tree.back().get_center();
      double root_radius = tree.back().get_radius();

      model_obstacle_centers.push_back(root_center);
      model_obstacle_radii.push_back(root_radius);
    }

    RCLCPP_INFO(this->get_logger(), "computing local workspace polygon");

    polygon local_workspace_polygon = compute_local_workspace_polygon(
        transform_result.transformed_position, model_obstacle_centers,
        model_obstacle_radii);

    polygon local_free_space_polygon;
    if (local_workspace_polygon.outer().size() < 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Local workspace polygon has less than 3 points, skipping "
                  "control");
      return;
    }
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(
        -robot_radius_);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy;
    bg::strategy::buffer::side_straight side_strategy;

    // erode the local workspace polygon to create free space
    multi_polygon local_free_space_multi;
    bg::buffer(local_workspace_polygon, local_free_space_multi,
               distance_strategy, side_strategy, join_strategy, end_strategy,
               point_strategy);

    // Extract the first polygon if buffer operation succeeded
    if (!local_free_space_multi.empty()) {
      local_free_space_polygon = local_free_space_multi[0];
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Buffer operation failed, using original polygon");
      local_free_space_polygon = local_workspace_polygon;
    }

    // Publish workspace markers before erosion for visualization
    publish_local_workspace_markers(local_workspace_polygon);

    // Publish freespace markers for visualization
    publish_freespace_markers(local_free_space_polygon);

    point robot_position_point(transform_result.transformed_position[0],
                               transform_result.transformed_position[1]);

    if (!has_subgoal_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "No subgoal available for control");
      return;
    }

    point subgoal_point(current_subgoal_.x, current_subgoal_.y);
    point local_goal_linear = compute_local_goal_linear(
        robot_position_point, transform_result.transformed_orientation,
        local_free_space_polygon, subgoal_point);

    point local_goal_angular =
        compute_local_goal(local_free_space_polygon, subgoal_point);

    publish_local_goal_marker(local_goal_angular);

    // Publish freespace markers for visualization
    publish_freespace_markers(local_free_space_polygon);

    // Compute the basis for the virtual control inputs
    double tV = (local_goal_linear.get<0>() -
                 transform_result.transformed_position[0]) *
                    cos(transform_result.transformed_orientation) +
                (local_goal_linear.get<1>() -
                 transform_result.transformed_position[1]) *
                    sin(transform_result.transformed_orientation);
    double tW1 = (local_goal_angular.get<0>() -
                  transform_result.transformed_position[0]) *
                     cos(transform_result.transformed_orientation) +
                 (local_goal_angular.get<1>() -
                  transform_result.transformed_position[1]) *
                     sin(transform_result.transformed_orientation);
    double tW2 = -(local_goal_angular.get<0>() -
                   transform_result.transformed_position[0]) *
                     sin(transform_result.transformed_orientation) +
                 (local_goal_angular.get<1>() -
                  transform_result.transformed_position[1]) *
                     cos(transform_result.transformed_orientation);

    // Compute the basis for transforming to actual control inputs

    double alpha1 = transform_result.alpha1;
    double alpha2 = transform_result.alpha2;
    double beta1 = transform_result.beta1;
    double beta2 = transform_result.beta2;

    double e_norm = sqrt(pow(alpha1, 2) + pow(alpha2, 2));
    double dksi_dpsi =
        MatrixDeterminant(transform_result.transformed_jacobian) /
        pow(e_norm, 2);
    double DksiCosSin = (alpha1 * beta1 + alpha2 * beta2) / pow(e_norm, 2);

    double linear_ctl_gain, angular_ctl_gain;
    std::vector<double> limit_check_vector_linear = {
        linear_gain_, linear_cmd_limit_ * e_norm / std::abs(tV),
        0.4 * angular_cmd_limit_ * dksi_dpsi * e_norm /
            std::abs(tV * DksiCosSin)};
    linear_ctl_gain = *std::min_element(limit_check_vector_linear.begin(),
                                        limit_check_vector_linear.end());
    std::vector<double> limit_check_vector_angular = {
        angular_gain_,
        0.6 * angular_cmd_limit_ * dksi_dpsi / std::abs(std::atan2(tW2, tW1))};

    angular_ctl_gain = *std::min_element(limit_check_vector_angular.begin(),
                                         limit_check_vector_angular.end());

    // Compute virtual and actual inputs
    double dV_virtual = linear_ctl_gain * tV;
    double linear_cmd = dV_virtual / e_norm;
    double dW_virtual = angular_ctl_gain * std::atan2(tW2, tW1);
    double angular_cmd = (dW_virtual - linear_cmd * DksiCosSin) / dksi_dpsi;

    // Check if robot is within tolerance of subgoal
    double distance_to_subgoal =
        sqrt(pow(current_subgoal_.x - current_position_.x, 2) +
             pow(current_subgoal_.y - current_position_.y, 2));

    if (distance_to_subgoal <= next_subgoal_tolerance_) {
      if (!subgoal_reached_) {
        subgoal_reached_pub_->publish(std_msgs::msg::Empty());
        subgoal_reached_ = true;
      }
    }

    if (distance_to_subgoal <= goal_tolerance_) {
      // Robot is within goal tolerance, stop movement
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(),
                  "Robot within goal tolerance (%.3f <= %.3f), stopping",
                  distance_to_subgoal, goal_tolerance_);
    } else {
      // Normal control commands
      cmd_vel.linear.x =
          std::max(-linear_cmd_limit_, std::min(linear_cmd, linear_cmd_limit_));
      cmd_vel.angular.z = std::max(-angular_cmd_limit_,
                                   std::min(angular_cmd, angular_cmd_limit_));
    }

    cmd_vel_pub_->publish(cmd_vel);
  }

  polygon
  compute_robot_voronoi_cell(const point &robot_pos,
                             const std::vector<point> &obstacle_centers) {
    // Start with the environment boundary as initial polygon
    polygon env_polygon;
    bg::append(
        env_polygon.outer(),
        bg::model::point<double, 2, bg::cs::cartesian>(env_x_min_, env_y_min_));
    bg::append(
        env_polygon.outer(),
        bg::model::point<double, 2, bg::cs::cartesian>(env_x_max_, env_y_min_));
    bg::append(
        env_polygon.outer(),
        bg::model::point<double, 2, bg::cs::cartesian>(env_x_max_, env_y_max_));
    bg::append(
        env_polygon.outer(),
        bg::model::point<double, 2, bg::cs::cartesian>(env_x_min_, env_y_max_));
    bg::append(env_polygon.outer(),
               bg::model::point<double, 2, bg::cs::cartesian>(
                   env_x_min_, env_y_min_)); // Close polygon
    bg::correct(env_polygon);

    // Erode the environment polygon by robot radius (same as merged obstacles)
    polygon result_polygon;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(
        -robot_radius_);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy;
    bg::strategy::buffer::side_straight side_strategy;

    multi_polygon eroded_env_multi;
    bg::buffer(env_polygon, eroded_env_multi, distance_strategy, side_strategy,
               join_strategy, end_strategy, point_strategy);

    // Use the eroded environment polygon if buffer operation succeeded
    if (!eroded_env_multi.empty()) {
      result_polygon = eroded_env_multi[0];
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Environment erosion failed, using original envelope");
      result_polygon = env_polygon;
    }

    // For each obstacle center, clip the polygon with the half-plane closer to
    // robot
    for (const auto &obstacle_center : obstacle_centers) {
      // Skip if obstacle is at robot position
      double dx = obstacle_center.get<0>() - robot_pos.get<0>();
      double dy = obstacle_center.get<1>() - robot_pos.get<1>();
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < 1e-9) {
        continue;
      }

      // Perpendicular bisector: points equidistant from robot and obstacle
      // The half-plane closer to robot is where: distance_to_robot <
      // distance_to_obstacle This is equivalent to: (x-rx)² + (y-ry)² < (x-ox)²
      // + (y-oy)² Simplifying: 2*(ox-rx)*x + 2*(oy-ry)*y < ox²+oy²-rx²-ry²

      double rx = robot_pos.get<0>();
      double ry = robot_pos.get<1>();
      double ox = obstacle_center.get<0>();
      double oy = obstacle_center.get<1>();

      // Create a large polygon representing the half-plane closer to robot
      // Half-plane: 2*(ox-rx)*x + 2*(oy-ry)*y < ox²+oy²-rx²-ry²
      double a = 2.0 * (ox - rx);
      double b = 2.0 * (oy - ry);
      double c = ox * ox + oy * oy - rx * rx - ry * ry;

      // Create a large rectangle and keep only points satisfying the half-plane
      // constraint
      double extend = 1000.0;
      std::vector<std::pair<double, double>> boundary_points = {
          {env_x_min_ - extend, env_y_min_ - extend},
          {env_x_max_ + extend, env_y_min_ - extend},
          {env_x_max_ + extend, env_y_max_ + extend},
          {env_x_min_ - extend, env_y_max_ + extend}};

      // Find which corner points satisfy the half-plane constraint
      std::vector<std::pair<double, double>> valid_points;
      for (const auto &p : boundary_points) {
        if (a * p.first + b * p.second < c) { // Robot side of bisector
          valid_points.push_back(p);
        }
      }

      // Add intersection points with boundary edges
      // Check intersection with each edge of the extended rectangle
      std::vector<std::pair<double, double>> intersections;

      // Bottom edge: y = env_y_min_ - extend
      if (std::abs(b) > 1e-9) {
        double y = env_y_min_ - extend;
        double x = (c - b * y) / a;
        if (x >= env_x_min_ - extend && x <= env_x_max_ + extend) {
          intersections.push_back({x, y});
        }
      }

      // Top edge: y = env_y_max_ + extend
      if (std::abs(b) > 1e-9) {
        double y = env_y_max_ + extend;
        double x = (c - b * y) / a;
        if (x >= env_x_min_ - extend && x <= env_x_max_ + extend) {
          intersections.push_back({x, y});
        }
      }

      // Left edge: x = env_x_min_ - extend
      if (std::abs(a) > 1e-9) {
        double x = env_x_min_ - extend;
        double y = (c - a * x) / b;
        if (y >= env_y_min_ - extend && y <= env_y_max_ + extend) {
          intersections.push_back({x, y});
        }
      }

      // Right edge: x = env_x_max_ + extend
      if (std::abs(a) > 1e-9) {
        double x = env_x_max_ + extend;
        double y = (c - a * x) / b;
        if (y >= env_y_min_ - extend && y <= env_y_max_ + extend) {
          intersections.push_back({x, y});
        }
      }

      // Combine valid corner points and intersections
      for (const auto &intersection : intersections) {
        valid_points.push_back(intersection);
      }

      // Create half-plane polygon if we have enough points
      if (valid_points.size() >= 3) {
        // Sort points counter-clockwise around centroid
        double cx = 0, cy = 0;
        for (const auto &p : valid_points) {
          cx += p.first;
          cy += p.second;
        }
        cx /= valid_points.size();
        cy /= valid_points.size();

        std::sort(valid_points.begin(), valid_points.end(),
                  [cx, cy](const std::pair<double, double> &a,
                           const std::pair<double, double> &b) {
                    return std::atan2(a.second - cy, a.first - cx) <
                           std::atan2(b.second - cy, b.first - cx);
                  });

        // Create polygon
        polygon half_plane;
        for (const auto &p : valid_points) {
          bg::append(half_plane.outer(),
                     bg::model::point<double, 2, bg::cs::cartesian>(p.first,
                                                                    p.second));
        }
        bg::append(half_plane.outer(),
                   bg::model::point<double, 2, bg::cs::cartesian>(
                       valid_points[0].first, valid_points[0].second)); // Close
        bg::correct(half_plane);

        // Intersect current result with this half-plane
        multi_polygon intersection_result;
        bg::intersection(result_polygon, half_plane, intersection_result);

        if (!intersection_result.empty()) {
          // Find the largest intersection component
          double max_area = 0;
          size_t best_idx = 0;
          for (size_t i = 0; i < intersection_result.size(); ++i) {
            double area = bg::area(intersection_result[i]);
            if (area > max_area) {
              max_area = area;
              best_idx = i;
            }
          }
          result_polygon = intersection_result[best_idx];
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "No intersection found with half-plane for obstacle at "
                      "(%.3f, %.3f)",
                      obstacle_center.get<0>(), obstacle_center.get<1>());
        }
      }
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "Computed robot Voronoi cell with %zu vertices",
                 result_polygon.outer().size());

    return result_polygon;
  }

  polygon compute_local_workspace_polygon(
      std::vector<double> robot_position_transformed,
      std::vector<point> &model_obstacle_centers,
      std::vector<double> &model_obstacle_radii) {

    point robot_position(robot_position_transformed[0],
                         robot_position_transformed[1]);

    // Get the points on the obstacle radius that are closest to the robot
    std::vector<point> interpolated_obstacle_centers;
    for (size_t i = 0; i < model_obstacle_centers.size(); ++i) {
      point obstacle_center = model_obstacle_centers[i];
      double obstacle_radius = model_obstacle_radii[i];

      if (obstacle_radius > 1e-9) { // Only interpolate if radius is non-zero
        line l;
        l.push_back(obstacle_center);
        l.push_back(robot_position);

        point result;
        bg::line_interpolate(l, obstacle_radius, result);
        interpolated_obstacle_centers.push_back(result);
      } else {
        // If radius is zero, use the obstacle center directly
        interpolated_obstacle_centers.push_back(obstacle_center);
      }
    }

    // Publish obstacle centers markers
    publish_obstacle_centers_markers(model_obstacle_centers);

    // Publish interpolated centers markers
    publish_interpolated_centers_markers(interpolated_obstacle_centers);

    // Compute robot's Voronoi cell directly using half-plane intersections
    polygon local_workspace_polygon = compute_robot_voronoi_cell(
        robot_position, interpolated_obstacle_centers);

    return local_workspace_polygon;
  }

  point custom_raycast(const polygon &poly, const point &ray_start,
                       const point &ray_direction) {
    // Normalize the ray direction
    double ray_length = sqrt(ray_direction.get<0>() * ray_direction.get<0>() +
                             ray_direction.get<1>() * ray_direction.get<1>());
    if (ray_length < 1e-9) {
      return ray_start;
    }

    point normalized_dir(ray_direction.get<0>() / ray_length,
                         ray_direction.get<1>() / ray_length);

    // Create a very long ray segment for intersection testing
    double max_distance = 1000.0;
    point ray_end(ray_start.get<0>() + max_distance * normalized_dir.get<0>(),
                  ray_start.get<1>() + max_distance * normalized_dir.get<1>());

    // Create line segment for the ray
    bg::model::segment<point> ray_segment(ray_start, ray_end);

    // Find intersection with polygon boundary
    double closest_distance = max_distance;
    point closest_intersection = ray_end;
    bool found_intersection = false;

    const auto &ring = poly.outer();
    if (ring.size() < 3) {
      return ray_start;
    }

    // Check intersection with each edge of the polygon
    for (size_t i = 0; i < ring.size() - 1; ++i) {
      point p1 = ring[i];
      point p2 = ring[i + 1];

      bg::model::segment<point> edge_segment(p1, p2);

      // Find intersection points between ray and edge
      std::vector<point> intersection_points;
      bg::intersection(ray_segment, edge_segment, intersection_points);

      // Check each intersection point
      for (const auto &intersection : intersection_points) {
        // Calculate distance from ray start to intersection
        double dx = intersection.get<0>() - ray_start.get<0>();
        double dy = intersection.get<1>() - ray_start.get<1>();
        double distance = sqrt(dx * dx + dy * dy);

        // Check if intersection is in the positive ray direction
        double dot_product =
            dx * normalized_dir.get<0>() + dy * normalized_dir.get<1>();

        if (dot_product > 1e-9 && distance < closest_distance) {
          closest_distance = distance;
          closest_intersection = intersection;
          found_intersection = true;
        }
      }
    }

    return found_intersection ? closest_intersection : ray_start;
  }

  line compute_linear_free_space(point robot_position, double robot_orientation,
                                 polygon local_free_space_polygon) {

    line free_space_line;
    if (bg::area(local_free_space_polygon) < 0.001) {
      free_space_line.push_back(robot_position);
      free_space_line.push_back(robot_position);
    } else {
      const double erosion_distance = 0.015; // Erosion distance in meters

      // Create eroded polygon using multi-polygon as intermediate
      std::vector<polygon> eroded_multipolygon;
      bg::strategy::buffer::distance_symmetric<double> distance_strategy(
          -erosion_distance);
      bg::strategy::buffer::join_round join_strategy;
      bg::strategy::buffer::end_round end_strategy;
      bg::strategy::buffer::point_circle circle_strategy;
      bg::strategy::buffer::side_straight side_strategy;
      bg::buffer(local_free_space_polygon, eroded_multipolygon,
                 distance_strategy, side_strategy, join_strategy, end_strategy,
                 circle_strategy);

      polygon eroded_polygon;
      if (!eroded_multipolygon.empty()) {
        eroded_polygon =
            eroded_multipolygon[0]; // Use first polygon from result
      }

      // Determine starting point for raycast
      point raycast_start = robot_position;

      // If robot is outside eroded polygon, project it onto eroded polygon
      // boundary
      if (!eroded_polygon.outer().empty() &&
          !bg::within(robot_position, eroded_polygon)) {
        if (bg::area(eroded_polygon) > 0.001) {
          ProjectionResultStruct projection =
              polydist(eroded_polygon, robot_position);
          raycast_start = projection.projected_point;
        }
      }

      free_space_line.push_back(robot_position);
      free_space_line.push_back(custom_raycast(
          local_free_space_polygon, raycast_start,
          point(cos(robot_orientation), sin(robot_orientation))));
    }

    // Publish visualization marker for the free space line
    publish_free_space_line_marker(free_space_line);

    return free_space_line;
  }

  point compute_local_goal_linear(point robot_position,
                                  double robot_orientation,
                                  polygon local_free_space_polygon,
                                  point goal) {
    line lfl = compute_linear_free_space(robot_position, robot_orientation,
                                         local_free_space_polygon);
    if (bg::is_empty(lfl)) {
      return robot_position;
    } else {
      ProjectionResultStruct projection_result = linedist(lfl, goal);
      return projection_result.projected_point;
    }
  }

  point compute_local_goal(polygon lfs, point goal) {
    if (bg::within(goal, lfs)) {
      return goal; // Goal is already within local free space
    } else {
      // Find the closest point on the polygon boundary
      double min_distance = std::numeric_limits<double>::max();
      point closest_point = goal;

      const auto &ring = lfs.outer();
      if (ring.size() < 3) {
        return goal; // Invalid polygon
      }

      // Check each edge of the polygon
      for (size_t i = 0; i < ring.size() - 1; ++i) {
        point p1 = ring[i];
        point p2 = ring[i + 1];

        // Vector from p1 to p2
        double dx = p2.get<0>() - p1.get<0>();
        double dy = p2.get<1>() - p1.get<1>();

        // Vector from p1 to goal
        double px = goal.get<0>() - p1.get<0>();
        double py = goal.get<1>() - p1.get<1>();

        // Project goal onto the infinite line through p1-p2
        double edge_length_sq = dx * dx + dy * dy;
        point projected_point;

        if (edge_length_sq < 1e-9) {
          // Degenerate edge (p1 == p2), use p1
          projected_point = point(p1.get<0>(), p1.get<1>());
        } else {
          // Parameter t for projection: 0 = p1, 1 = p2
          double t = (px * dx + py * dy) / edge_length_sq;

          // Clamp t to [0,1] to stay within the edge segment
          t = std::max(0.0, std::min(1.0, t));

          // Compute the closest point on this edge
          projected_point = point(p1.get<0>() + t * dx, p1.get<1>() + t * dy);
        }

        // Check if this is the closest point so far
        double distance = bg::distance(goal, projected_point);
        if (distance < min_distance) {
          min_distance = distance;
          closest_point = projected_point;
        }
      }

      return closest_point;
    }
  }

  void publish_freespace_markers(const polygon &local_free_space_polygon) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    // Create a marker for the freespace polygon
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "freespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    marker.scale.x = 0.05; // Line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    // Convert polygon points to marker points
    for (const auto &boost_point : local_free_space_polygon.outer()) {
      geometry_msgs::msg::Point point;
      point.x = bg::get<0>(boost_point);
      point.y = bg::get<1>(boost_point);
      point.z = 0.0;
      marker.points.push_back(point);
    }

    // Close the polygon by adding the first point at the end
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points[0]);
    }

    marker_array.markers.push_back(marker);
    freespace_markers_pub_->publish(marker_array);
  }

  void publish_local_workspace_markers(const polygon &local_workspace_polygon) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "local_workspace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.06;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 0.9;

    for (const auto &boost_point : local_workspace_polygon.outer()) {
      geometry_msgs::msg::Point point;
      point.x = boost_point.get<0>();
      point.y = boost_point.get<1>();
      point.z = 0.0;
      marker.points.push_back(point);
    }

    if (!marker.points.empty()) {
      marker.points.push_back(marker.points[0]);
    }

    marker_array.markers.push_back(marker);
    local_workspace_markers_pub_->publish(marker_array);
  }

  void publish_envelope_markers(const polygon &envelope_polygon) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    // Create a marker for the envelope polygon
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "envelope";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    marker.scale.x = 0.08; // Line width (slightly thicker than freespace)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    // Convert envelope points to marker points
    for (const auto &boost_point : envelope_polygon.outer()) {
      geometry_msgs::msg::Point point;
      point.x = boost_point.get<0>();
      point.y = boost_point.get<1>();
      point.z = 0.0;
      marker.points.push_back(point);
    }

    // Close the polygon by adding the first point at the end
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points[0]);
    }

    marker_array.markers.push_back(marker);
    envelope_markers_pub_->publish(marker_array);
  }

  void
  publish_obstacle_centers_markers(const std::vector<point> &obstacle_centers) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < obstacle_centers.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "obstacle_centers";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = obstacle_centers[i].get<0>();
      marker.pose.position.y = obstacle_centers[i].get<1>();
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0; // Magenta for obstacle centers
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    obstacle_centers_markers_pub_->publish(marker_array);
  }

  void publish_interpolated_centers_markers(
      const std::vector<point> &interpolated_centers) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < interpolated_centers.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "interpolated_centers";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = interpolated_centers[i].get<0>();
      marker.pose.position.y = interpolated_centers[i].get<1>();
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;

      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0; // Cyan for interpolated centers
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    interpolated_centers_markers_pub_->publish(marker_array);
  }

  void
  publish_merged_polygon_markers(const std::vector<polygon> &merged_polygons) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < merged_polygons.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "merged_polygons";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;

      const auto &poly = merged_polygons[i];
      for (const auto &pt : poly.outer()) {
        geometry_msgs::msg::Point point;
        point.x = bg::get<0>(pt);
        point.y = bg::get<1>(pt);
        point.z = 0.0;
        marker.points.push_back(point);
      }

      if (!marker.points.empty()) {
        marker.points.push_back(marker.points[0]);
      }

      marker_array.markers.push_back(marker);
    }

    merged_polygon_markers_pub_->publish(marker_array);
  }

  void publish_clipped_polygon_markers(
      const std::vector<polygon> &clipped_polygons) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < clipped_polygons.size(); ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "clipped_polygons";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.06; // Slightly thicker than merged polygons
      marker.color.r = 0.0;
      marker.color.g = 0.8; // Green color to distinguish from merged (magenta)
      marker.color.b = 0.2;
      marker.color.a = 1.0; // Fully opaque

      const auto &poly = clipped_polygons[i];
      for (const auto &pt : poly.outer()) {
        geometry_msgs::msg::Point point;
        point.x = bg::get<0>(pt);
        point.y = bg::get<1>(pt);
        point.z = 0.0;
        marker.points.push_back(point);
      }

      if (!marker.points.empty()) {
        marker.points.push_back(marker.points[0]);
      }

      marker_array.markers.push_back(marker);
    }

    clipped_polygon_markers_pub_->publish(marker_array);
  }

  void publish_local_goal_marker(const point &local_goal) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "local_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set position
    marker.pose.position.x = local_goal.get<0>();
    marker.pose.position.y = local_goal.get<1>();
    marker.pose.position.z = 0.1; // Slightly elevated for visibility
    marker.pose.orientation.w = 1.0;

    // Set scale (sphere radius)
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set color (bright yellow)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Set lifetime (marker persists until updated)
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    local_goal_marker_pub_->publish(marker);
  }

  void publish_transformed_position_marker(
      const std::vector<double> &transformed_position) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "transformed_position";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set position
    marker.pose.position.x = transformed_position[0];
    marker.pose.position.y = transformed_position[1];
    marker.pose.position.z = 0.2; // Elevated for visibility
    marker.pose.orientation.w = 1.0;

    // Set scale (sphere radius)
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set color (bright orange to distinguish from other markers)
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Set lifetime (marker persists until updated)
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    transformed_position_marker_pub_->publish(marker);
  }

  void publish_free_space_line_marker(const line &free_space_line) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "free_space_line";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set line properties
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // Line width

    // Set color (bright cyan for visibility)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Set lifetime (marker persists until updated)
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    // Add points to the line
    for (const auto &point : free_space_line) {
      geometry_msgs::msg::Point p;
      p.x = point.get<0>();
      p.y = point.get<1>();
      p.z = 0.05; // Slightly elevated for visibility
      marker.points.push_back(p);
    }

    free_space_line_marker_pub_->publish(marker);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReactiveNavigationNode>();
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
