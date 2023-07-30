#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

//#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "ctrl_toolbox/DataStructs.h"
//#include "ctrl_toolbox/HelperFunctions.h"
#include "data_structs.hpp"
//#include "obstacle_avoidance/Utils.hpp"
#include <stack>
#include <set>
#include <math.h>
#include <eigen3/Eigen/Eigen>
//#include <Eigen/Geometry>
#include <utility>

class path_planner {
private:
  VehicleInfo v_info_;
  ObstaclesInfo obss_info_;

  // Compute the costs of a newly created Node,
  //  depending on the time to reach the last node, goal position and time shift wrt last node
  void UpdateCosts(Node& node, double costToReachNode, const Eigen::Vector2d& goal, double timeShift);

  // Find the bb vertexes position depending on the vehicle position
  //void ComputeVertexes(Obstacle& obs, Eigen::Vector2d vehicle_position, double time, std::vector<Vertex>& vertexes);

  // Given an obstacle, find which vertexes are visible from the vehicle
  //void SetVisibleVertexes();

  // Given some obstacle vertexes, find the intercept points
  void ComputeInterceptPoints(const Obstacle& obstacle, const Eigen::Vector2d& vehicle_position, std::vector<Vertex>& vertexes);

  // Check if the path between start and goal collide with any obstacle
  bool CheckCollision(Node start, Node goal);

public:
  // vehicle start position and obstacles information are supposed to be taken in the same time instant.
  // the user can always compute the update position/obstacles info before creating the class

  path_planner(VehicleInfo v_info, ObstaclesInfo obss_info)
    : v_info_(std::move(v_info)), obss_info_(std::move(obss_info))
    {}

  // Compute the path to reach the goal and fills the waypoints stack
  bool ComputePath(const Eigen::Vector2d& goal, std::stack<Waypoint>& waypoints);

};
#endif
