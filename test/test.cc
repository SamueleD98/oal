#include <memory>
#include <string>

#include "oal/path_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

int main(int, char**)
{
  VehicleInfo v_info;
  Eigen::Vector2d position = {0,0};
  v_info.position = position;
  v_info.speed = 2;

  Obstacle obs1;
  Obstacle obs2;
  obs1.id_ = "1";
  Eigen::Vector2d position1 = {1,0};
  obs1.position_ = position1;
  obs1.heading_ = 2;
  obs1.speed_ = 0;
  obs1.dim_x_ = 1;
  obs1.dim_y_ = 2;
  obs2.id_ = "2";
  Eigen::Vector2d position2 = {8,2};
  obs2.position_ = position2;
  obs2.heading_ = 0;
  obs2.speed_ = 0;
  obs2.dim_x_ = 1;
  obs2.dim_y_ = 2;

  ObstaclesInfo obss_info;
  obss_info.obstacles.push_back(obs1);
  obss_info.obstacles.push_back(obs2);

  path_planner planner(v_info, obss_info);

  Eigen::Vector2d goal = {10,10};
  std::stack<Waypoint> waypoints;
  std::cout << "Calling library!" << std::endl;
  if(planner.ComputePath(goal, waypoints)){
    //std::cout << "Found!" << std::endl;
    std::cout << "Found!" << std::endl;
    while (!waypoints.empty()) {
      Waypoint wp = waypoints.top();
      waypoints.pop();
      //RCLCPP_INFO(this->get_logger(), "Pos: %d!", wp.position);
      //RCLCPP_INFO(this->get_logger(), "Time: %f!", wp.time);
      std::cout << "Pos: " << wp.position << "Time: " << wp.time << std::endl;
    }
  }
  std::cout << "Not found." << std::endl;
  //RCLCPP_INFO(this->get_logger(), "Not found.");
}
