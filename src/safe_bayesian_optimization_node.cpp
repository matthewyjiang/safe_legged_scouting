#include "polygeom_lib.h"
#include "safe_bayesian_optimization/msg/polygon_array.hpp"
#include "trusses_custom_interfaces/srv/get_terrain_map_with_uncertainty.hpp"
#include "std_msgs/msg/int32.hpp"
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <boost/geometry.hpp>
#include <chrono>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/detail/envelope/interface.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <vector>

namespace bg = boost::geometry;

class OptimizerNode : public rclcpp::Node {
public:
  OptimizerNode() : Node("optimizer_node") {
    // Declare parameters
    this->declare_parameter("opt.beta", 2.0);
    this->declare_parameter("opt.f_max", 0.0);
    this->declare_parameter("opt.lipschitz", 1.0);
    this->declare_parameter("terrain_map.resolution",
                            std::vector<double>{0.1, 0.1});
    this->declare_parameter("debug.publish_debug_image", false);
    this->declare_parameter("opt.subgoal_erosion", 0.2);
    this->declare_parameter("robot.radius", 0.5);
    this->declare_parameter("polygon.simplification_tolerance", 0.1);

    // Read parameters
    beta_ = this->get_parameter("opt.beta").as_double();
    f_max_ = this->get_parameter("opt.f_max").as_double();
    lipschitz_L_ = this->get_parameter("opt.lipschitz").as_double();
    auto resolution_vector =
        this->get_parameter("terrain_map.resolution").as_double_array();
    if (resolution_vector.size() != 2) {
      RCLCPP_ERROR(this->get_logger(),
                   "terrain_map.resolution must be an array of 2 values "
                   "[x_resolution, y_resolution]");
      throw std::runtime_error("Invalid terrain_map.resolution parameter");
    }
    terrain_x_resolution_ = resolution_vector[0];
    terrain_y_resolution_ = resolution_vector[1];
    publish_debug_image_ =
        this->get_parameter("debug.publish_debug_image").as_bool();
    subgoal_erosion_ = this->get_parameter("opt.subgoal_erosion").as_double();
    robot_radius_ = this->get_parameter("robot.radius").as_double();
    simplification_tolerance_ = this->get_parameter("polygon.simplification_tolerance").as_double();

    // Create service clients
    terrain_map_client_ = this->create_client<
        trusses_custom_interfaces::srv::GetTerrainMapWithUncertainty>(
        "get_terrain_map_with_uncertainty");

    // Create subscribers
    goal_point_sub_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "goal_point", 10,
            std::bind(&OptimizerNode::goal_point_callback, this,
                      std::placeholders::_1));
    
    spatial_data_size_sub_ =
        this->create_subscription<std_msgs::msg::Int32>(
            "spatial_data_size", 10,
            std::bind(&OptimizerNode::spatial_data_size_callback, this,
                      std::placeholders::_1));

    // Create publishers
    current_subgoal_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        "current_subgoal", 10);

    subgoal_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "subgoal_marker", 10);
    polygons_pub_ =
        this->create_publisher<safe_bayesian_optimization::msg::PolygonArray>(
            "polygon_array", 10);

    envelope_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>(
        "envelope_polygon", 10);

    // Create concave polygon markers publisher
    concave_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/concave_markers", 10);

    // Create projected goal marker publisher
    projected_goal_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/projected_goal_marker", 10);

    // Only create debug image publisher if enabled
    if (publish_debug_image_) {
      debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
          "debug_polygons_image", 10);
    }

    // Initialize last spatial data size
    last_spatial_data_size_ = 0;

    // No goal received yet
    goal_received_ = false;
    confidence_initialized_ = false;
    safe_set_initialized_ = false;

    RCLCPP_INFO(this->get_logger(), "Optimizer Node Initialized");
  }

private:
  Eigen::VectorXd mu_;  // Mean vector
  Eigen::VectorXd std_; // Standard deviation vector

  Eigen::Matrix<double, Eigen::Dynamic, 2> D_; // Parameter Set;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> S_;   // Safe set
  Eigen::Matrix<double, Eigen::Dynamic, 2> Q_; // Confidence intervals
  Eigen::VectorXd C_low_;  // Monotonic lower bounds
  Eigen::VectorXd C_high_; // Monotonic upper bounds

  double beta_;
  double f_max_;
  double lipschitz_L_;
  double subgoal_erosion_;
  double robot_radius_;
  double simplification_tolerance_;

  // Spatial data monitoring
  rclcpp::Client<trusses_custom_interfaces::srv::GetTerrainMapWithUncertainty>::
      SharedPtr terrain_map_client_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      goal_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      spatial_data_size_sub_;
  int last_spatial_data_size_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_subgoal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      subgoal_marker_pub_;
  rclcpp::Publisher<safe_bayesian_optimization::msg::PolygonArray>::SharedPtr
      polygons_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr envelope_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      concave_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      projected_goal_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  // Parameters from config
  double terrain_x_resolution_;
  double terrain_y_resolution_;
  bool publish_debug_image_;

  // Terrain dimensions from service response
  int terrain_width_cells_;
  int terrain_height_cells_;

  // Current goal point
  geometry_msgs::msg::PointStamped current_goal_;
  bool goal_received_;

  // Store eroded concave polygon for subgoal projection
  bg::model::polygon<bg::model::d2::point_xy<double>> eroded_concave_polygon_;

  // Timing variables for terrain map request
  std::chrono::steady_clock::time_point terrain_request_start_time_;

  bool confidence_initialized_;
  bool safe_set_initialized_;

  void ComputeSets() {
    UpdateMonotonicConfidenceIntervals();
    UpdateSafeSetWithLipschitz();
  }

  void InitializeConfidenceBoundsIfNeeded() {
    if (C_low_.size() == mu_.size()) {
      return;
    }

    const double neg_inf = -std::numeric_limits<double>::infinity();
    const double pos_inf = std::numeric_limits<double>::infinity();

    C_low_.resize(mu_.size());
    C_high_.resize(mu_.size());
    C_low_.setConstant(neg_inf);
    C_high_.setConstant(pos_inf);

    S_.resize(mu_.size());
    S_.setConstant(false);

    confidence_initialized_ = true;
    safe_set_initialized_ = false;
  }

  void UpdateMonotonicConfidenceIntervals() {
    InitializeConfidenceBoundsIfNeeded();

    const double sqrt_beta = std::sqrt(beta_);
    const Eigen::VectorXd confidence = sqrt_beta * std_;

    const Eigen::VectorXd Q_low = mu_ - confidence;
    const Eigen::VectorXd Q_high = mu_ + confidence;

    Q_.col(0) = Q_low;
    Q_.col(1) = Q_high;

    C_low_ = C_low_.cwiseMax(Q_low);
    C_high_ = C_high_.cwiseMin(Q_high);
  }

  void UpdateSafeSetWithLipschitz() {
    if (S_.rows() == 0 || D_.rows() == 0) {
      return;
    }

    if (lipschitz_L_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Lipschitz constant <= 0; safe set will not expand.");
      return;
    }

    const int n_points = D_.rows();
    std::vector<char> safe_flags(n_points, 0);

    std::vector<int> anchor_indices;
    anchor_indices.reserve(n_points);

    for (int i = 0; i < n_points; ++i) {
      if (S_(i) || C_high_(i) <= f_max_) {
        safe_flags[i] = 1;
        anchor_indices.push_back(i);
      }
    }

    const int width = terrain_width_cells_;
    const int height = terrain_height_cells_;
    const bool has_grid = (width > 0 && height > 0 &&
                           static_cast<int>(width * height) == n_points);

    if (has_grid) {
      const double min_x = D_.col(0).minCoeff();
      const double max_x = D_.col(0).maxCoeff();
      const double min_y = D_.col(1).minCoeff();
      const double max_y = D_.col(1).maxCoeff();
      const double cell_width =
          (width > 1) ? (max_x - min_x) / (width - 1) : terrain_x_resolution_;
      const double cell_height =
          (height > 1) ? (max_y - min_y) / (height - 1) : terrain_y_resolution_;
      const double max_grid_dist =
          std::hypot((width - 1) * cell_width, (height - 1) * cell_height);

      for (int anchor : anchor_indices) {
        const double u_anchor = C_high_(anchor);
        const double margin = f_max_ - u_anchor;
        if (margin < 0.0) {
          continue;
        }

        const double radius = margin / lipschitz_L_;
        if (radius >= max_grid_dist) {
          for (int i = 0; i < n_points; ++i) {
            safe_flags[i] = 1;
          }
          break;
        }
        const double radius_sq = radius * radius;

        const int anchor_i = anchor / height;
        const int anchor_j = anchor % height;
        const int max_i_offset =
            static_cast<int>(std::floor(radius / cell_width));
        const int max_j_offset =
            static_cast<int>(std::floor(radius / cell_height));

        const int i_min = std::max(0, anchor_i - max_i_offset);
        const int i_max = std::min(width - 1, anchor_i + max_i_offset);
        const int j_min = std::max(0, anchor_j - max_j_offset);
        const int j_max = std::min(height - 1, anchor_j + max_j_offset);

        for (int i = i_min; i <= i_max; ++i) {
          const double dx = (i - anchor_i) * cell_width;
          const double dx_sq = dx * dx;
          for (int j = j_min; j <= j_max; ++j) {
            const int idx = i * height + j;
            if (safe_flags[idx]) {
              continue;
            }
            const double dy = (j - anchor_j) * cell_height;
            const double dist_sq = dx_sq + dy * dy;
            if (dist_sq <= radius_sq) {
              safe_flags[idx] = 1;
            }
          }
        }
      }
    } else {
      for (int anchor : anchor_indices) {
        const double u_anchor = C_high_(anchor);
        const double margin = f_max_ - u_anchor;
        if (margin < 0.0) {
          continue;
        }

        const double radius = margin / lipschitz_L_;
        const double radius_sq = radius * radius;

        for (int j = 0; j < n_points; ++j) {
          if (safe_flags[j]) {
            continue;
          }
          const double dist_sq = (D_.row(anchor) - D_.row(j)).squaredNorm();
          if (dist_sq <= radius_sq) {
            safe_flags[j] = 1;
          }
        }
      }
    }

    for (int i = 0; i < n_points; ++i) {
      S_(i) = safe_flags[i];
    }
  }

  std::vector<int> FindSafetyContourIndices() {
    if (D_.rows() == 0 || S_.rows() == 0) {
      RCLCPP_WARN(this->get_logger(), "D_ or S_ is empty");
      return {};
    }

    // count S_ that is true

    int safe_count = S_.count();
    RCLCPP_INFO(this->get_logger(), "Number of safe points: %d", safe_count);
    RCLCPP_INFO(this->get_logger(), "Total points in S_: %ld", S_.size());

    // Find grid bounds
    int min_x = static_cast<int>(D_.col(0).minCoeff());
    int max_x = static_cast<int>(D_.col(0).maxCoeff());
    int min_y = static_cast<int>(D_.col(1).minCoeff());
    int max_y = static_cast<int>(D_.col(1).maxCoeff());

    // Use stored grid dimensions from service response
    int width = terrain_width_cells_;
    int height = terrain_height_cells_;

    RCLCPP_INFO(this->get_logger(), "Using grid dimensions: %dx%d", width,
                height);

    // Create binary image from safety data
    cv::Mat safety_image = cv::Mat::zeros(height, width, CV_8UC1);

    for (int i = 0; i < D_.rows(); ++i) {
      // Get indices in the grid from D_, where the coordinates are not
      // necessarily integers. so we need to scale them

      int x = static_cast<int>((D_(i, 0) - min_x) / (max_x - min_x) * width);
      int y = static_cast<int>((D_(i, 1) - min_y) / (max_y - min_y) * height);

      if (x >= 0 && x < width && y >= 0 && y < height) {
        safety_image.at<uchar>(y, x) = S_(i) ? 255 : 0;
      }
    }

    // Find contours with hierarchy to properly handle holes
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(safety_image, contours, hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_SIMPLE);

    RCLCPP_INFO(this->get_logger(), "Found %lu contours", contours.size());

    // Convert contour points back to original indices using a hash map for O(1)
    // lookup
    std::unordered_map<int, std::unordered_map<int, int>> coord_to_index;
    for (int i = 0; i < D_.rows(); ++i) {
      int x = static_cast<int>((D_(i, 0) - min_x) / (max_x - min_x) * width);
      int y = static_cast<int>((D_(i, 1) - min_y) / (max_y - min_y) * height);
      if (x >= 0 && x < width && y >= 0 && y < height) {
        coord_to_index[x][y] = i;
      }
    }

    std::vector<int> contour_indices;
    contour_indices.reserve(contours.size() *
                            50); // Reserve reasonable capacity

    for (const auto &contour : contours) {
      for (const auto &point : contour) {
        // O(1) lookup using scaled coordinates
        auto x_it = coord_to_index.find(point.x);
        if (x_it != coord_to_index.end()) {
          auto y_it = x_it->second.find(point.y);
          if (y_it != x_it->second.end()) {
            contour_indices.push_back(y_it->second);
          }
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "Found %lu contour points",
                contour_indices.size());
    return contour_indices;
  }

  int GetNextSubgoal() {
    // Get frontier indices
    const std::vector<int> frontier_indices = FindSafetyContourIndices();

    if (frontier_indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "No frontier points found");
      return -1;
    }

    // Extract frontier points
    const size_t num_frontiers = frontier_indices.size();
    Eigen::MatrixXd frontier_points(num_frontiers, 2);
    Eigen::VectorXd frontier_confidence_width(num_frontiers);

    for (size_t i = 0; i < num_frontiers; ++i) {
      const int idx = frontier_indices[i];
      frontier_points.row(i) = D_.row(idx);
      frontier_confidence_width(i) = Q_(idx, 1) - Q_(idx, 0);
    }

    if (!goal_received_) {
      double best_confidence_width = -1.0;
      int best_index = -1;

      for (size_t i = 0; i < num_frontiers; ++i) {
        double conf_width = frontier_confidence_width(i);

        if (conf_width > best_confidence_width) {
          best_confidence_width = conf_width;
          best_index = static_cast<int>(i);
        }
      }

      if (best_index < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to select frontier point by uncertainty");
        return -1;
      }

      return frontier_indices[best_index];
    }

    const Eigen::VectorXd goal_eigen =
        (Eigen::VectorXd(2) << current_goal_.point.x, current_goal_.point.y)
            .finished();
    const Eigen::VectorXd distances =
        (frontier_points.rowwise() - goal_eigen.transpose()).rowwise().norm();

    // Sort points by distance to goal
    std::vector<std::pair<double, size_t>> distance_pairs;
    for (size_t i = 0; i < num_frontiers; ++i) {
      distance_pairs.emplace_back(distances(i), i);
    }
    std::sort(distance_pairs.begin(), distance_pairs.end());

    // Take top 25% of closest points
    size_t top_quarter_size = std::max(size_t(1), distance_pairs.size() / 4);

    // Find the point with best confidence width among the closest points
    double best_confidence_width = -1.0;
    int best_index = -1;

    for (size_t i = 0; i < top_quarter_size; ++i) {
      size_t frontier_idx = distance_pairs[i].second;
      double conf_width = frontier_confidence_width(frontier_idx);

      if (conf_width > best_confidence_width) {
        best_confidence_width = conf_width;
        best_index = static_cast<int>(frontier_idx);
      }
    }

    if (best_index < 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to select frontier point near goal");
      return -1;
    }

    return frontier_indices[best_index];
  }

  void spatial_data_size_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    int current_size = msg->data;
    
    // Only process if the size has changed
    if (current_size != last_spatial_data_size_) {
      RCLCPP_INFO(this->get_logger(),
                  "Spatial data size changed: %d -> %d",
                  last_spatial_data_size_, current_size);
      
      last_spatial_data_size_ = current_size;
      
      // Request terrain map when size changes
      request_terrain_map();
    }
  }

  void request_terrain_map() {
    if (!terrain_map_client_->wait_for_service(std::chrono::seconds(0))) {
      RCLCPP_WARN(this->get_logger(), "Terrain map service not available");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Requesting terrain map...");

    auto request =
        std::make_shared<trusses_custom_interfaces::srv::
                             GetTerrainMapWithUncertainty::Request>();
    // Use the resolution array field from the service interface
    request->resolution = {static_cast<float>(terrain_x_resolution_),
                           static_cast<float>(terrain_y_resolution_)};

    RCLCPP_INFO(this->get_logger(),
                "Requesting terrain map with resolution: [%.2f, %.2f] "
                "(service will calculate dimensions from data bounds)",
                terrain_x_resolution_, terrain_y_resolution_);

    // Start timing the terrain map request
    terrain_request_start_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Starting terrain map request timing...");

    // Use callback-based async call
    terrain_map_client_->async_send_request(
        request,
        [this](rclcpp::Client<
               trusses_custom_interfaces::srv::GetTerrainMapWithUncertainty>::
                   SharedFuture future) {
          // Calculate terrain map request duration
          auto end_time = std::chrono::steady_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
              end_time - terrain_request_start_time_);
          
          RCLCPP_INFO(this->get_logger(), 
                      "Terrain map request completed in %ld ms", duration.count());
          
          auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Received terrain map: %s",
                        response->message.c_str());

            // Store grid dimensions for later use
            terrain_width_cells_ = response->n_width_cells;
            terrain_height_cells_ = response->n_height_cells;

            // Process the terrain map data here
            process_terrain_map(response);

          } else {
            RCLCPP_WARN(this->get_logger(), "Terrain map request failed: %s",
                        response->message.c_str());
          }
        });
  }

  void process_terrain_map(
      const std::shared_ptr<const trusses_custom_interfaces::srv::
                                GetTerrainMapWithUncertainty::Response>
          response) {
    // Update the parameter set D_, mean mu_, and std_ with the new terrain map
    // data
    const size_t num_points = response->x_coords.size();

    D_.resize(num_points, 2);
    mu_.resize(num_points);
    std_.resize(num_points);
    Q_.resize(num_points, 2);
    S_.resize(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      D_(i, 0) = response->x_coords[i];
      D_(i, 1) = response->y_coords[i];
      mu_(i) = response->values[i];
      std_(i) = response->uncertainties[i];
    }

    // Recompute sets with new data
    ComputeSets();

    publish_obstacle_polygons();

    // Check if the goal itself is safe by checking if it's within the eroded
    // polygon
    bg::model::d2::point_xy<double> goal_point(current_goal_.point.x,
                                               current_goal_.point.y);

    geometry_msgs::msg::Point subgoal;
    if (goal_received_ && bg::within(goal_point, eroded_concave_polygon_)) {
      // Goal is safe, use it directly as the subgoal
      subgoal.x = current_goal_.point.x;
      subgoal.y = current_goal_.point.y;
      subgoal.z = 0.0;

      RCLCPP_INFO(
          this->get_logger(),
          "Goal is within eroded safe region, using goal as subgoal: (%f, %f)",
          subgoal.x, subgoal.y);
    } else {
      if (!goal_received_) {
        RCLCPP_INFO(this->get_logger(),
                    "No goal available; selecting frontier with highest uncertainty");
      }
      // Goal is not safe, find frontier subgoal and project it
      const int next_subgoal_index = GetNextSubgoal();
      if (next_subgoal_index >= 0) {
        // Get the raw subgoal from the terrain map
        double raw_x = response->x_coords[next_subgoal_index];
        double raw_y = response->y_coords[next_subgoal_index];

        // Convert boost polygon to polygeom_lib polygon format
        polygon eroded_poly_for_projection;
        for (const auto &boost_point : eroded_concave_polygon_.outer()) {
          point polygeom_point(boost_point.get<0>(), boost_point.get<1>());
          eroded_poly_for_projection.outer().push_back(polygeom_point);
        }

        bg::correct(eroded_poly_for_projection);

        RCLCPP_INFO(this->get_logger(),
                    "Next subgoal index: %d, coordinates: (%f, %f)",
                    next_subgoal_index, raw_x, raw_y);

        // Create point from raw subgoal coordinates
        point raw_subgoal_point(raw_x, raw_y);

        // Project the subgoal onto the eroded safe region using polydist
        ProjectionResultStruct projection_result =
            polydist(eroded_poly_for_projection, raw_subgoal_point);

        RCLCPP_INFO(this->get_logger(),
                    "Projected subgoal point: (%f, %f), distance: %f",
                    projection_result.projected_point.get<0>(),
                    projection_result.projected_point.get<1>(),
                    projection_result.dist);

        // Create final subgoal message from projected point
        subgoal.x = projection_result.projected_point.get<0>();
        subgoal.y = projection_result.projected_point.get<1>();
        subgoal.z = 0.0;

        // Publish visualization of the projection when a goal is available
        if (goal_received_) {
          publish_projected_goal_marker(current_goal_.point, subgoal,
                                        projection_result.dist);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Published projected subgoal: (%f, %f) (distance: %f)",
                    subgoal.x, subgoal.y, projection_result.dist);
      } else {
        RCLCPP_WARN(this->get_logger(), "No valid subgoal found");
        return;
      }
    }

    current_subgoal_pub_->publish(subgoal);

    // Publish subgoal marker for visualization
    publish_subgoal_marker(subgoal);
  }

  bg::model::polygon<bg::model::d2::point_xy<double>>
  create_concave_polygon_from_points(const std::vector<int> &point_indices) {
    RCLCPP_INFO(this->get_logger(),
                "[POLYGON] Creating concave polygon from %zu points",
                point_indices.size());

    // Use CGAL Alpha Shape instead of concaveman
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
    typedef CGAL::Alpha_shape_face_base_2<K> Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
    typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;
    typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;
    typedef K::Point_2 Point_2;

    std::vector<Point_2> cgal_points;
    cgal_points.reserve(point_indices.size());
    for (int idx : point_indices) {
      cgal_points.emplace_back(D_(idx, 0), D_(idx, 1));
    }

    if (cgal_points.size() < 3) {
      RCLCPP_WARN(
          this->get_logger(),
          "[POLYGON] Insufficient points (%zu) for polygon, returning empty",
          cgal_points.size());
      return bg::model::polygon<bg::model::d2::point_xy<double>>();
    }

    // Create alpha shape
    Alpha_shape_2 alpha_shape(cgal_points.begin(), cgal_points.end());
    alpha_shape.set_mode(Alpha_shape_2::REGULARIZED);

    // Find optimal alpha value - start with infinity (convex hull) and work down
    alpha_shape.set_alpha(std::numeric_limits<double>::infinity());
    
    // Get all possible alpha values in descending order
    std::vector<double> alpha_values;
    for (auto alpha_it = alpha_shape.alpha_begin(); alpha_it != alpha_shape.alpha_end(); ++alpha_it) {
      if (*alpha_it > 0) {
        alpha_values.push_back(*alpha_it);
      }
    }
    std::sort(alpha_values.rbegin(), alpha_values.rend()); // Sort in descending order
    
    // Find the largest alpha that includes all points
    bool found_valid_alpha = false;
    for (double alpha_val : alpha_values) {
      alpha_shape.set_alpha(alpha_val);
      
      // Count vertices in the alpha shape
      int vertex_count = 0;
      for (auto vertex_it = alpha_shape.alpha_shape_vertices_begin();
           vertex_it != alpha_shape.alpha_shape_vertices_end(); ++vertex_it) {
        vertex_count++;
      }
      
      // If this alpha includes all or most points (at least 90%), use it
      if (vertex_count >= static_cast<int>(cgal_points.size() * 0.9)) {
        found_valid_alpha = true;
        RCLCPP_INFO(this->get_logger(), 
                    "[POLYGON] Using alpha=%f, includes %d/%zu vertices", 
                    alpha_val, vertex_count, cgal_points.size());
        break;
      }
    }
    
    if (!found_valid_alpha) {
      // Fallback to a very large alpha to ensure we get most points
      alpha_shape.set_alpha(std::numeric_limits<double>::infinity());
      RCLCPP_WARN(this->get_logger(), 
                  "[POLYGON] Using convex hull (alpha=infinity) as fallback");
    }

    // Collect boundary edges
    std::vector<typename Alpha_shape_2::Edge> boundary_edges;
    for (auto edge_it = alpha_shape.alpha_shape_edges_begin();
         edge_it != alpha_shape.alpha_shape_edges_end(); ++edge_it) {
      boundary_edges.push_back(*edge_it);
    }

    bg::model::polygon<bg::model::d2::point_xy<double>> concave_polygon;

    if (!boundary_edges.empty()) {
      std::vector<Point_2> ordered_boundary;
      std::map<Point_2, std::vector<Point_2>> adjacency_map;

      // Build adjacency map from boundary edges
      for (const auto &edge : boundary_edges) {
        auto face = edge.first;
        int idx = edge.second;
        Point_2 p1 = face->vertex((idx + 1) % 3)->point();
        Point_2 p2 = face->vertex((idx + 2) % 3)->point();
        adjacency_map[p1].push_back(p2);
        adjacency_map[p2].push_back(p1);
      }

      // Traverse the boundary to create ordered polygon
      if (!adjacency_map.empty()) {
        std::set<Point_2> visited;
        Point_2 start_point = adjacency_map.begin()->first;
        Point_2 current = start_point;

        do {
          ordered_boundary.push_back(current);
          visited.insert(current);
          Point_2 next = current;
          for (const auto &neighbor : adjacency_map[current]) {
            if (visited.find(neighbor) == visited.end() ||
                (neighbor == start_point && ordered_boundary.size() > 2)) {
              next = neighbor;
              break;
            }
          }
          current = next;
        } while (current != start_point &&
                 visited.find(current) == visited.end() &&
                 ordered_boundary.size() < adjacency_map.size());
      }

      // Convert ordered boundary to boost geometry polygon
      for (const auto &point : ordered_boundary) {
        bg::append(concave_polygon.outer(),
                   bg::model::d2::point_xy<double>(CGAL::to_double(point.x()),
                                                   CGAL::to_double(point.y())));
      }
    } else {
      // Fallback: use all points
      for (const auto &point : cgal_points) {
        bg::append(concave_polygon.outer(),
                   bg::model::d2::point_xy<double>(CGAL::to_double(point.x()),
                                                   CGAL::to_double(point.y())));
      }
    }

    bg::correct(concave_polygon);
    
    // Simplify the concave polygon with tolerance 0.1
    bg::model::polygon<bg::model::d2::point_xy<double>> simplified_polygon;
    bg::simplify(concave_polygon, simplified_polygon, 0.1);
    
    return simplified_polygon;
  }

  void publish_obstacle_polygons() {
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Starting publish_obstacle_polygons()");

    int safe_count = S_.count();
    RCLCPP_INFO(this->get_logger(), "[MAIN] Total safe points: %d", safe_count);

    if (safe_count == 0) {
      RCLCPP_WARN(this->get_logger(),
                  "[MAIN] No safe points available for polygon generation");
      return;
    }

    // Create polygons with holes using OpenCV contours (more efficient than Alpha Shapes)
    RCLCPP_INFO(this->get_logger(), "[MAIN] Creating safe region polygons from contours...");

    // Find grid bounds
    double min_x = D_.col(0).minCoeff();
    double max_x = D_.col(0).maxCoeff();
    double min_y = D_.col(1).minCoeff();
    double max_y = D_.col(1).maxCoeff();

    // Use stored grid dimensions from service response
    int width = terrain_width_cells_;
    int height = terrain_height_cells_;

    RCLCPP_INFO(this->get_logger(), "[MAIN] Grid bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f], size=%dx%d",
                min_x, max_x, min_y, max_y, width, height);

    // Create binary image from safety data
    cv::Mat safety_image = cv::Mat::zeros(height, width, CV_8UC1);

    for (int i = 0; i < D_.rows(); ++i) {
      int x = static_cast<int>((D_(i, 0) - min_x) / (max_x - min_x) * width);
      int y = static_cast<int>((D_(i, 1) - min_y) / (max_y - min_y) * height);

      if (x >= 0 && x < width && y >= 0 && y < height) {
        safety_image.at<uchar>(y, x) = S_(i) ? 255 : 0;
      }
    }

    // Find contours with hierarchy to detect holes
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(safety_image, contours, hierarchy,
                     cv::RETR_CCOMP,  // Retrieves all contours and organizes them into a two-level hierarchy
                     cv::CHAIN_APPROX_SIMPLE);  // Compresses segments for efficiency

    RCLCPP_INFO(this->get_logger(), "[MAIN] Found %zu contours from OpenCV", contours.size());

    // Build polygons with holes from contour hierarchy
    // hierarchy[i] = [next, previous, first_child, parent]
    // parent == -1: outer boundary (safe region)
    // parent >= 0: hole (unsafe island inside safe region)
    std::vector<bg::model::polygon<bg::model::d2::point_xy<double>>>
        concave_polygons;
    std::vector<std::array<double, 2>> all_boundaries;

    for (size_t i = 0; i < contours.size(); ++i) {
      // Process only outer boundaries (no parent)
      if (hierarchy[i][3] == -1) {
        bg::model::polygon<bg::model::d2::point_xy<double>> poly;

        // Add outer boundary
        for (const auto& pt : contours[i]) {
          double world_x = min_x + (pt.x / (double)width) * (max_x - min_x);
          double world_y = min_y + (pt.y / (double)height) * (max_y - min_y);
          bg::append(poly.outer(), bg::model::d2::point_xy<double>(world_x, world_y));
          all_boundaries.push_back({world_x, world_y});
        }

        RCLCPP_INFO(this->get_logger(), "[MAIN] Outer boundary %zu has %zu points",
                    i, contours[i].size());

        // Find and add holes (children of this outer boundary)
        int hole_count = 0;
        for (size_t j = 0; j < contours.size(); ++j) {
          if (hierarchy[j][3] == (int)i) {  // This contour's parent is current outer boundary
            poly.inners().resize(poly.inners().size() + 1);
            for (const auto& pt : contours[j]) {
              double world_x = min_x + (pt.x / (double)width) * (max_x - min_x);
              double world_y = min_y + (pt.y / (double)height) * (max_y - min_y);
              bg::append(poly.inners().back(),
                        bg::model::d2::point_xy<double>(world_x, world_y));
            }
            hole_count++;
            RCLCPP_INFO(this->get_logger(), "[MAIN]   Hole %d has %zu points",
                        hole_count, contours[j].size());
          }
        }

        // Ensure proper orientation (outer CCW, holes CW)
        bg::correct(poly);

        if (!poly.outer().empty()) {
          concave_polygons.push_back(poly);
          RCLCPP_INFO(this->get_logger(),
                      "[MAIN] Created polygon with %zu outer points and %zu holes",
                      poly.outer().size(), poly.inners().size());
        }
      }
    }

    if (concave_polygons.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[MAIN] No valid polygons created from contours");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Successfully created %zu safe region polygons using OpenCV contours",
                concave_polygons.size());

    // Apply conservative simplification: erode THEN simplify
    // This guarantees safe regions only shrink → obstacles only grow
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Applying conservative polygon simplification (tolerance=%.3fm)...",
                simplification_tolerance_);

    double safety_erosion = 1.5 * simplification_tolerance_;  // Conservative: erode by 1.5x tolerance

    std::vector<bg::model::polygon<bg::model::d2::point_xy<double>>>
        simplified_polygons;

    for (size_t i = 0; i < concave_polygons.size(); ++i) {
      size_t orig_outer_vertices = concave_polygons[i].outer().size();
      size_t orig_inner_vertices = 0;
      for (const auto& inner : concave_polygons[i].inners()) {
        orig_inner_vertices += inner.size();
      }

      // Process outer boundary and holes separately for conservative simplification
      bg::strategy::buffer::join_round join_strategy;
      bg::strategy::buffer::end_round end_strategy;
      bg::strategy::buffer::point_circle point_strategy;
      bg::strategy::buffer::side_straight side_strategy;

      // Step 1: Erode outer boundary (shrink safe region)
      bg::model::polygon<bg::model::d2::point_xy<double>> outer_only;
      outer_only.outer() = concave_polygons[i].outer();
      bg::correct(outer_only);

      bg::strategy::buffer::distance_symmetric<double> erosion_distance(-safety_erosion);
      bg::model::multi_polygon<bg::model::polygon<bg::model::d2::point_xy<double>>>
          eroded_outer_multi;
      bg::buffer(outer_only, eroded_outer_multi, erosion_distance,
                 side_strategy, join_strategy, end_strategy, point_strategy);

      if (eroded_outer_multi.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "[MAIN] Polygon %zu outer boundary disappeared after erosion - skipping", i);
        continue;
      }

      // Step 2: Dilate holes (expand unsafe regions inside)
      std::vector<bg::model::polygon<bg::model::d2::point_xy<double>>> dilated_holes;

      for (const auto& hole : concave_polygons[i].inners()) {
        bg::model::polygon<bg::model::d2::point_xy<double>> hole_poly;
        hole_poly.outer() = hole;
        bg::correct(hole_poly);

        bg::strategy::buffer::distance_symmetric<double> dilation_distance(safety_erosion);
        bg::model::multi_polygon<bg::model::polygon<bg::model::d2::point_xy<double>>>
            dilated_hole_multi;
        bg::buffer(hole_poly, dilated_hole_multi, dilation_distance,
                   side_strategy, join_strategy, end_strategy, point_strategy);

        if (!dilated_hole_multi.empty()) {
          dilated_holes.push_back(dilated_hole_multi[0]);
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "[MAIN] Polygon %zu: outer eroded, %zu holes dilated",
                  i, dilated_holes.size());

      // Step 3: Simplify outer boundary
      bg::model::polygon<bg::model::d2::point_xy<double>> simplified_outer;
      bg::simplify(eroded_outer_multi[0], simplified_outer, simplification_tolerance_);

      if (simplified_outer.outer().empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "[MAIN] Polygon %zu outer disappeared after simplification - skipping", i);
        continue;
      }

      // Step 4: Simplify holes
      std::vector<bg::model::polygon<bg::model::d2::point_xy<double>>> simplified_holes;
      for (const auto& dilated_hole : dilated_holes) {
        bg::model::polygon<bg::model::d2::point_xy<double>> simplified_hole;
        bg::simplify(dilated_hole, simplified_hole, simplification_tolerance_);

        if (!simplified_hole.outer().empty()) {
          simplified_holes.push_back(simplified_hole);
        }
      }

      // Step 5: Combine outer boundary with holes
      bg::model::polygon<bg::model::d2::point_xy<double>> final_poly;
      final_poly.outer() = simplified_outer.outer();

      // Add simplified holes as inner rings
      for (const auto& hole : simplified_holes) {
        final_poly.inners().push_back(hole.outer());
      }

      bg::correct(final_poly);

      if (!final_poly.outer().empty()) {
        size_t final_outer_vertices = final_poly.outer().size();
        size_t final_inner_vertices = 0;
        for (const auto& inner : final_poly.inners()) {
          final_inner_vertices += inner.size();
        }

        simplified_polygons.push_back(final_poly);

        RCLCPP_INFO(this->get_logger(),
                    "[MAIN] Polygon %zu: vertices %zu→%zu (outer), %zu→%zu (inner holes), reduction: %.1f%%",
                    i, orig_outer_vertices, final_outer_vertices,
                    orig_inner_vertices, final_inner_vertices,
                    100.0 * (1.0 - (double)(final_outer_vertices + final_inner_vertices) /
                             (double)(orig_outer_vertices + orig_inner_vertices)));
      }
    }

    if (simplified_polygons.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[MAIN] All polygons disappeared after simplification - using originals");
      simplified_polygons = concave_polygons;
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "[MAIN] Successfully simplified %zu→%zu polygons with safety guarantee",
                  concave_polygons.size(), simplified_polygons.size());
    }

    // Use simplified polygons for remaining operations
    concave_polygons = simplified_polygons;

    // Find the largest polygon for subgoal projection (or use first one)
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Finding largest polygon for subgoal projection...");
    size_t largest_polygon_idx = 0;
    double largest_area = 0.0;
    for (size_t i = 0; i < concave_polygons.size(); ++i) {
      double area = bg::area(concave_polygons[i]);
      RCLCPP_INFO(this->get_logger(), "[MAIN] Polygon %zu area: %f", i, area);
      if (area > largest_area) {
        largest_area = area;
        largest_polygon_idx = i;
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Selected polygon %zu as largest (area: %f)",
                largest_polygon_idx, largest_area);

    // Erode the largest concave polygon for safe subgoal placement
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Eroding largest polygon for subgoal projection...");
    bg::strategy::buffer::distance_symmetric<double> erosion_distance(
        -subgoal_erosion_);
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle point_strategy;
    bg::strategy::buffer::side_straight side_strategy;

    bg::model::multi_polygon<
        bg::model::polygon<bg::model::d2::point_xy<double>>>
        eroded_multi;
    bg::buffer(concave_polygons[largest_polygon_idx], eroded_multi,
               erosion_distance, side_strategy, join_strategy, end_strategy,
               point_strategy);

    if (!eroded_multi.empty()) {
      eroded_concave_polygon_ = eroded_multi[0];
      RCLCPP_INFO(this->get_logger(), "[MAIN] Successfully eroded largest "
                                      "concave polygon for subgoal projection");
    } else {
      eroded_concave_polygon_ = concave_polygons[largest_polygon_idx];
      RCLCPP_WARN(
          this->get_logger(),
          "[MAIN] Erosion failed, using original largest concave polygon");
    }

    // Publish concave polygon markers for visualization (all polygons)
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Publishing visualization markers...");
    publish_multiple_concave_markers(concave_polygons);

    // Create envelope from all concave polygons
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Creating envelope from all polygons...");
    bg::model::box<bg::model::d2::point_xy<double>> envelope_box;
    bool first_polygon = true;
    for (size_t i = 0; i < concave_polygons.size(); ++i) {
      const auto &polygon = concave_polygons[i];
      bg::model::box<bg::model::d2::point_xy<double>> poly_box;
      bg::envelope(polygon, poly_box);

      if (first_polygon) {
        envelope_box = poly_box;
        first_polygon = false;
        RCLCPP_INFO(
            this->get_logger(),
            "[MAIN] Initial envelope: min(%.2f,%.2f) max(%.2f,%.2f)",
            poly_box.min_corner().get<0>(), poly_box.min_corner().get<1>(),
            poly_box.max_corner().get<0>(), poly_box.max_corner().get<1>());
      } else {
        // Expand envelope to include this polygon
        bg::expand(envelope_box, poly_box);
        RCLCPP_INFO(this->get_logger(),
                    "[MAIN] Expanded envelope for polygon %zu", i);
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[MAIN] Final envelope: min(%.2f,%.2f) max(%.2f,%.2f)",
        envelope_box.min_corner().get<0>(), envelope_box.min_corner().get<1>(),
        envelope_box.max_corner().get<0>(), envelope_box.max_corner().get<1>());

    // Publish envelope polygon
    geometry_msgs::msg::Polygon envelope_msg;
    geometry_msgs::msg::Point32 min_corner, max_corner;
    min_corner.x = envelope_box.min_corner().get<0>();
    min_corner.y = envelope_box.min_corner().get<1>();
    max_corner.x = envelope_box.max_corner().get<0>();
    max_corner.y = envelope_box.max_corner().get<1>();

    envelope_msg.points.push_back(min_corner);
    geometry_msgs::msg::Point32 bottom_right;
    bottom_right.x = max_corner.x;
    bottom_right.y = min_corner.y;
    envelope_msg.points.push_back(bottom_right);
    envelope_msg.points.push_back(max_corner);
    geometry_msgs::msg::Point32 top_left;
    top_left.x = min_corner.x;
    top_left.y = max_corner.y;
    envelope_msg.points.push_back(top_left);

    envelope_pub_->publish(envelope_msg);

    // Create obstacle polygons by subtracting ALL concave polygons from
    // envelope
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Creating obstacle polygons by subtracting concave "
                "polygons from envelope...");
    bg::model::multi_polygon<
        bg::model::polygon<bg::model::d2::point_xy<double>>>
        obstacle_polygons;

    // Start with the envelope box, eroded by robot radius
    bg::model::multi_polygon<
        bg::model::polygon<bg::model::d2::point_xy<double>>>
        current_obstacles;
    bg::model::polygon<bg::model::d2::point_xy<double>> envelope_polygon;
    bg::append(
        envelope_polygon.outer(),
        bg::model::d2::point_xy<double>(envelope_box.min_corner().get<0>(),
                                        envelope_box.min_corner().get<1>()));
    bg::append(
        envelope_polygon.outer(),
        bg::model::d2::point_xy<double>(envelope_box.max_corner().get<0>(),
                                        envelope_box.min_corner().get<1>()));
    bg::append(
        envelope_polygon.outer(),
        bg::model::d2::point_xy<double>(envelope_box.max_corner().get<0>(),
                                        envelope_box.max_corner().get<1>()));
    bg::append(
        envelope_polygon.outer(),
        bg::model::d2::point_xy<double>(envelope_box.min_corner().get<0>(),
                                        envelope_box.max_corner().get<1>()));
    bg::correct(envelope_polygon);
    
    // Erode the envelope polygon by robot radius
    bg::strategy::buffer::distance_symmetric<double> envelope_erosion_distance(
        -robot_radius_);
    bg::strategy::buffer::join_round envelope_join_strategy;
    bg::strategy::buffer::end_round envelope_end_strategy;
    bg::strategy::buffer::point_circle envelope_point_strategy;
    bg::strategy::buffer::side_straight envelope_side_strategy;

    bg::model::multi_polygon<
        bg::model::polygon<bg::model::d2::point_xy<double>>>
        eroded_envelope_multi;
    bg::buffer(envelope_polygon, eroded_envelope_multi,
               envelope_erosion_distance, envelope_side_strategy, 
               envelope_join_strategy, envelope_end_strategy,
               envelope_point_strategy);

    if (!eroded_envelope_multi.empty()) {
      current_obstacles.push_back(eroded_envelope_multi[0]);
      RCLCPP_INFO(this->get_logger(),
                  "[MAIN] Successfully eroded envelope polygon by robot radius: %f",
                  robot_radius_);
    } else {
      current_obstacles.push_back(envelope_polygon);
      RCLCPP_WARN(
          this->get_logger(),
          "[MAIN] Envelope erosion failed, using original envelope polygon");
    }
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Starting with envelope polygon (%zu obstacle polygons)",
                current_obstacles.size());

    // Subtract each concave polygon from the current obstacle set
    for (size_t i = 0; i < concave_polygons.size(); ++i) {
      const auto &concave_polygon = concave_polygons[i];
      RCLCPP_INFO(this->get_logger(),
                  "[MAIN] Subtracting concave polygon %zu from obstacles...",
                  i);

      bg::model::multi_polygon<
          bg::model::polygon<bg::model::d2::point_xy<double>>>
          temp_obstacles;
      bg::difference(current_obstacles, concave_polygon, temp_obstacles);

      RCLCPP_INFO(
          this->get_logger(),
          "[MAIN] After subtracting polygon %zu: %zu -> %zu obstacle polygons",
          i, current_obstacles.size(), temp_obstacles.size());

      current_obstacles = temp_obstacles;
    }

    obstacle_polygons = current_obstacles;
    RCLCPP_INFO(this->get_logger(), "[MAIN] Final obstacle polygons: %zu",
                obstacle_polygons.size());

    // Publish obstacle polygons
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Publishing %zu obstacle polygons...",
                obstacle_polygons.size());
    safe_bayesian_optimization::msg::PolygonArray polygon_array_msg;
    for (size_t i = 0; i < obstacle_polygons.size(); ++i) {
      const auto &polygon = obstacle_polygons[i];
      geometry_msgs::msg::Polygon msg_polygon;
      for (const auto &point : polygon.outer()) {
        geometry_msgs::msg::Point32 p;
        p.x = point.get<0>();
        p.y = point.get<1>();
        msg_polygon.points.push_back(p);
      }
      polygon_array_msg.polygons.push_back(msg_polygon);
      RCLCPP_INFO(this->get_logger(),
                  "[MAIN] Obstacle polygon %zu has %zu points", i,
                  polygon.outer().size());
    }
    polygons_pub_->publish(polygon_array_msg);
    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Successfully published obstacle polygons");

    // Publish debug visualization image if enabled
    if (publish_debug_image_ && debug_image_pub_) {
      RCLCPP_INFO(this->get_logger(),
                  "[MAIN] Publishing debug visualization image...");
      publish_debug_image(obstacle_polygons, all_boundaries);
    }

    RCLCPP_INFO(this->get_logger(),
                "[MAIN] Completed publish_obstacle_polygons()");
  }

  void
  publish_debug_image(const bg::model::multi_polygon<
                          bg::model::polygon<bg::model::d2::point_xy<double>>>
                          &obstacle_polygons,
                      const std::vector<std::array<double, 2>> &safe_hull) {
    // Calculate image bounds from data points
    if (D_.rows() == 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No data points available for debug image");
      return;
    }

    double x_min = D_.col(0).minCoeff();
    double x_max = D_.col(0).maxCoeff();
    double y_min = D_.col(1).minCoeff();
    double y_max = D_.col(1).maxCoeff();

    // Image size - use higher resolution for better visualization
    int img_width = 800;
    int img_height = 600;

    // Create debug image
    cv::Mat debug_img = cv::Mat::zeros(img_height, img_width, CV_8UC3);

    // Helper function to convert world coordinates to image coordinates
    auto world_to_image = [&](double world_x, double world_y) -> cv::Point {
      int img_x =
          static_cast<int>((world_x - x_min) / (x_max - x_min) * img_width);
      int img_y = static_cast<int>((1.0 - (world_y - y_min) / (y_max - y_min)) *
                                   img_height);
      return cv::Point(img_x, img_y);
    };

    // Draw concave hull of safe set using pre-computed hull
    if (safe_hull.size() > 2) {
      std::vector<cv::Point> hull_img_points;
      hull_img_points.reserve(safe_hull.size()); // Reserve capacity

      for (const auto &point : safe_hull) {
        cv::Point img_point = world_to_image(point[0], point[1]);
        // Only add points within image bounds
        if (img_point.x >= 0 && img_point.x < img_width && img_point.y >= 0 &&
            img_point.y < img_height) {
          hull_img_points.push_back(img_point);
        }
      }

      if (hull_img_points.size() > 2) {
        const cv::Point *pts = hull_img_points.data();
        int npts = hull_img_points.size();
        cv::fillPoly(debug_img, &pts, &npts, 1, cv::Scalar(0, 150, 0));
        cv::polylines(debug_img, hull_img_points, true, cv::Scalar(0, 255, 0),
                      2);
      }
    }

    // Draw safe region as circles

    for (int i = 0; i < D_.rows(); ++i) {
      if (S_(i)) {
        cv::Point img_point = world_to_image(D_(i, 0), D_(i, 1));
        if (img_point.x >= 0 && img_point.x < img_width && img_point.y >= 0 &&
            img_point.y < img_height) {
          cv::circle(debug_img, img_point, 3, cv::Scalar(255, 0, 0), -1);
        }
      }
    }

    // Draw obstacle polygons in blue
    for (const auto &polygon : obstacle_polygons) {
      const auto &outer_ring = polygon.outer();
      if (outer_ring.size() < 3)
        continue; // Skip invalid polygons

      // Draw outer boundary
      std::vector<cv::Point> cv_points;
      cv_points.reserve(outer_ring.size()); // Reserve capacity

      for (const auto &point : outer_ring) {
        cv::Point img_point = world_to_image(point.get<0>(), point.get<1>());
        cv_points.push_back(img_point);
      }

      if (cv_points.size() > 2) {
        const cv::Point *pts = cv_points.data();
        int npts = static_cast<int>(cv_points.size());
        cv::fillPoly(debug_img, &pts, &npts, 1, cv::Scalar(255, 100, 0));
        cv::polylines(debug_img, cv_points, true, cv::Scalar(255, 255, 0), 2);
      }

      // Draw holes in obstacle polygons (safe regions inside obstacles) if any
      for (const auto &inner_ring : polygon.inners()) {
        if (inner_ring.size() < 3)
          continue;

        std::vector<cv::Point> inner_cv_points;
        inner_cv_points.reserve(inner_ring.size());

        for (const auto &point : inner_ring) {
          cv::Point img_point = world_to_image(point.get<0>(), point.get<1>());
          inner_cv_points.push_back(img_point);
        }

        if (inner_cv_points.size() > 2) {
          // Fill holes with green (safe regions inside obstacles)
          const cv::Point *inner_pts = inner_cv_points.data();
          int inner_npts = static_cast<int>(inner_cv_points.size());
          cv::fillPoly(debug_img, &inner_pts, &inner_npts, 1, cv::Scalar(0, 150, 0));
          cv::polylines(debug_img, inner_cv_points, true, cv::Scalar(0, 255, 0), 2);
        }
      }
    }

    // // Draw current goal in magenta if available
    // if (current_goal_.point.x != 0.0 || current_goal_.point.y != 0.0) {
    //   cv::Point goal_img = world_to_image(current_goal_.point.x,
    //   current_goal_.point.y); if (goal_img.x >= 0 && goal_img.x < img_width
    //   &&
    //       goal_img.y >= 0 && goal_img.y < img_height) {
    //     cv::circle(debug_img, goal_img, 8, cv::Scalar(255, 0, 255), -1);
    //     cv::circle(debug_img, goal_img, 12, cv::Scalar(255, 255, 255), 2);
    //   }
    // }

    // Add legend
    cv::putText(debug_img, "Safe concave hull (green)", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(debug_img, "Obstacles (blue)", cv::Point(10, 55),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 100, 0), 2);
    cv::putText(debug_img, "Goal (magenta)", cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);

    // Convert OpenCV image to ROS image message
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "map";

    sensor_msgs::msg::Image::SharedPtr img_msg =
        cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();

    debug_image_pub_->publish(*img_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published debug visualization image");
  }

  void
  goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    current_goal_.point = msg->point;
    goal_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "New goal received: (%.2f, %.2f), requesting terrain map update",
                msg->point.x, msg->point.y);

    // Request terrain map to recompute subgoal with new goal
    request_terrain_map();
  }

  void publish_subgoal_marker(const geometry_msgs::msg::Point &subgoal) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "subgoal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = subgoal.x;
    marker.pose.position.y = subgoal.y;
    marker.pose.position.z = subgoal.z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    subgoal_marker_pub_->publish(marker);
  }

  void publish_concave_markers(
      const bg::model::polygon<bg::model::d2::point_xy<double>>
          &concave_polygon) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    int marker_id = 0;

    // Create a marker for the outer boundary
    auto outer_marker = visualization_msgs::msg::Marker();
    outer_marker.header.frame_id = "map";
    outer_marker.header.stamp = this->now();
    outer_marker.ns = "concave_outer";
    outer_marker.id = marker_id++;
    outer_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outer_marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    outer_marker.scale.x = 0.06; // Line width
    outer_marker.color.r = 0.0;
    outer_marker.color.g = 0.0;
    outer_marker.color.b = 1.0;
    outer_marker.color.a = 0.8;

    // Convert concave polygon outer boundary points to marker points
    for (const auto &boost_point : concave_polygon.outer()) {
      geometry_msgs::msg::Point point;
      point.x = boost_point.get<0>();
      point.y = boost_point.get<1>();
      point.z = 0.0;
      outer_marker.points.push_back(point);
    }

    // Close the polygon by adding the first point at the end
    if (!outer_marker.points.empty()) {
      outer_marker.points.push_back(outer_marker.points[0]);
    }

    marker_array.markers.push_back(outer_marker);

    // Create markers for holes (unsafe regions inside)
    for (size_t hole_idx = 0; hole_idx < concave_polygon.inners().size(); ++hole_idx) {
      const auto &hole = concave_polygon.inners()[hole_idx];

      auto hole_marker = visualization_msgs::msg::Marker();
      hole_marker.header.frame_id = "map";
      hole_marker.header.stamp = this->now();
      hole_marker.ns = "concave_holes";
      hole_marker.id = marker_id++;
      hole_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      hole_marker.action = visualization_msgs::msg::Marker::ADD;

      // Set hole marker properties - use red to indicate unsafe regions
      hole_marker.scale.x = 0.06; // Line width
      hole_marker.color.r = 1.0;  // Red for unsafe
      hole_marker.color.g = 0.0;
      hole_marker.color.b = 0.0;
      hole_marker.color.a = 0.8;

      // Convert hole points to marker points
      for (const auto &boost_point : hole) {
        geometry_msgs::msg::Point point;
        point.x = boost_point.get<0>();
        point.y = boost_point.get<1>();
        point.z = 0.0;
        hole_marker.points.push_back(point);
      }

      // Close the hole by adding the first point at the end
      if (!hole_marker.points.empty()) {
        hole_marker.points.push_back(hole_marker.points[0]);
      }

      marker_array.markers.push_back(hole_marker);
    }

    concave_markers_pub_->publish(marker_array);
  }

  void publish_multiple_concave_markers(
      const std::vector<bg::model::polygon<bg::model::d2::point_xy<double>>>
          &concave_polygons) {
    auto marker_array = visualization_msgs::msg::MarkerArray();

    // Color palette for different polygons
    std::vector<std::array<float, 3>> colors = {
        {0.0, 0.0, 1.0}, // Blue
        {0.0, 1.0, 0.0}, // Green
        {1.0, 0.0, 0.0}, // Red
        {1.0, 1.0, 0.0}, // Yellow
        {1.0, 0.0, 1.0}, // Magenta
        {0.0, 1.0, 1.0}, // Cyan
        {0.5, 0.0, 0.5}, // Purple
        {1.0, 0.5, 0.0}  // Orange
    };

    int marker_id = 0;

    for (size_t i = 0; i < concave_polygons.size(); ++i) {
      const auto &concave_polygon = concave_polygons[i];

      // Draw outer boundary (safe region boundary)
      auto outer_marker = visualization_msgs::msg::Marker();
      outer_marker.header.frame_id = "map";
      outer_marker.header.stamp = this->now();
      outer_marker.ns = "concave_outer";
      outer_marker.id = marker_id++;
      outer_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outer_marker.action = visualization_msgs::msg::Marker::ADD;

      // Set marker properties
      outer_marker.scale.x = 0.06; // Line width
      auto &color = colors[i % colors.size()];
      outer_marker.color.r = color[0];
      outer_marker.color.g = color[1];
      outer_marker.color.b = color[2];
      outer_marker.color.a = 0.8;

      // Convert concave polygon outer boundary points to marker points
      for (const auto &boost_point : concave_polygon.outer()) {
        geometry_msgs::msg::Point point;
        point.x = boost_point.get<0>();
        point.y = boost_point.get<1>();
        point.z = 0.0;
        outer_marker.points.push_back(point);
      }

      // Close the polygon by adding the first point at the end
      if (!outer_marker.points.empty()) {
        outer_marker.points.push_back(outer_marker.points[0]);
      }

      marker_array.markers.push_back(outer_marker);

      // Draw holes (unsafe regions inside safe region)
      for (size_t hole_idx = 0; hole_idx < concave_polygon.inners().size(); ++hole_idx) {
        const auto &hole = concave_polygon.inners()[hole_idx];

        auto hole_marker = visualization_msgs::msg::Marker();
        hole_marker.header.frame_id = "map";
        hole_marker.header.stamp = this->now();
        hole_marker.ns = "concave_holes";
        hole_marker.id = marker_id++;
        hole_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        hole_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set hole marker properties - use red to indicate unsafe regions
        hole_marker.scale.x = 0.06; // Line width
        hole_marker.color.r = 1.0;  // Red for unsafe
        hole_marker.color.g = 0.0;
        hole_marker.color.b = 0.0;
        hole_marker.color.a = 0.8;

        // Convert hole points to marker points
        for (const auto &boost_point : hole) {
          geometry_msgs::msg::Point point;
          point.x = boost_point.get<0>();
          point.y = boost_point.get<1>();
          point.z = 0.0;
          hole_marker.points.push_back(point);
        }

        // Close the hole by adding the first point at the end
        if (!hole_marker.points.empty()) {
          hole_marker.points.push_back(hole_marker.points[0]);
        }

        marker_array.markers.push_back(hole_marker);
      }
    }

    concave_markers_pub_->publish(marker_array);
  }

  void
  publish_projected_goal_marker(const geometry_msgs::msg::Point &original_goal,
                                const geometry_msgs::msg::Point &projected_goal,
                                double projection_distance) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->get_clock()->now();
    line_marker.ns = "projected_goal";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::msg::Marker::ADD;

    // Line from original goal to projected goal
    line_marker.points.push_back(original_goal);
    line_marker.points.push_back(projected_goal);

    line_marker.scale.x = 0.08; // Line width
    line_marker.color.r = 1.0;  // Red line
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 0.9;

    projected_goal_marker_pub_->publish(line_marker);

    // Also publish arrow marker showing projection direction
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = "map";
    arrow_marker.header.stamp = this->get_clock()->now();
    arrow_marker.ns = "projected_goal";
    arrow_marker.id = 1;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;

    arrow_marker.points.push_back(original_goal);
    arrow_marker.points.push_back(projected_goal);

    arrow_marker.scale.x = 0.05; // Shaft diameter
    arrow_marker.scale.y = 0.1;  // Head diameter
    arrow_marker.scale.z = 0.1;  // Head length
    arrow_marker.color.r = 0.8;
    arrow_marker.color.g = 0.2;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 0.8;

    // Small delay to ensure line is published first
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    projected_goal_marker_pub_->publish(arrow_marker);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OptimizerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
