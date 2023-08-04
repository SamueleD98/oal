
#include "oal/path_planner.hpp"

int main(int, char**)
{
  VehicleInfo v_info;
  Eigen::Vector2d position = {10,0};
  v_info.position = position;
  v_info.speed = 2;

  Obstacle obs1;
  Obstacle obs2;
  obs1.id_ = "1";
  Eigen::Vector2d position1 = {10,3};
  obs1.position_ = position1;
  obs1.heading_ = 0;
  obs1.speed_ = 0;
  obs1.dim_x_ = 0.3;
  obs1.dim_y_ = 0.3;

  obs2.id_ = "2";
  Eigen::Vector2d position2 = {10,4.1};
  obs2.position_ = position2;
  obs2.heading_ = 0;
  obs2.speed_ = 0;
  obs2.dim_x_ = 0.4;
  obs2.dim_y_ = 0.4;

  ObstaclesInfo obss_info;
  obss_info.obstacles.push_back(obs1);
  obss_info.obstacles.push_back(obs2);

  path_planner planner(v_info, obss_info);

  Eigen::Vector2d goal = {10,5};
  std::stack<Node> waypoints;
  std::cout << "Calling library!" << std::endl;
  if(planner.ComputePath(goal, waypoints)){
    //std::cout << "Found!" << std::endl;
    std::cout << "Found!" << std::endl;
    while (!waypoints.empty()) {
      Node wp = waypoints.top();
      waypoints.pop();
      //std::cout << "Pos: " << wp.position << std::endl;
      //std::cout << "Time: " << wp.time << std::endl;
    }
  }else{
    std::cout << "Not found." << std::endl;
  }
}
