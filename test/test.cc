
#include "oal/path_planner.hpp"

int main(int, char **) {


  VehicleInfo v_info;
  Eigen::Vector2d position = {10, 0};
  v_info.position = position;
  v_info.speed = 2;

  // Obstacle constructor arguments: string id, Eigen::Vector2d position, double heading, double speed, double dim_x, double dim_y, double max_bb_ratio, double safety_bb_ratio
  //Obstacle obs1 = Obstacle("1", {10, 4}, 3.14 / 2, 0.8, 0.5, 0.5, 2, 1.6);
  //Obstacle obs2 = Obstacle("2", {10, 4.09}, 3.14, 0.5, 0.4, 0.4, 2, 1.6);
  //Obstacle obs3 = Obstacle("3", {11, 0.5}, 3.14 * 3 / 4, 1.8, 0.2, 0.2, 2, 1.6);
  //Obstacle obs4 = Obstacle("4", {5, 0.5}, 3.14 / 3, 2, 0.2, 0.2, 2, 1.6);

  Obstacle obs1 = Obstacle("1", {10, 9}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs2 = Obstacle("2", {10, 11}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs3 = Obstacle("3", {11, 10}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs4 = Obstacle("4", {9, 10}, 0, 0, 0.5, 0.5, 2, 1.6);

  ObstaclesInfo obss_info;
  obss_info.obstacles.push_back(obs1);
  obss_info.obstacles.push_back(obs2);
  obss_info.obstacles.push_back(obs3);
  obss_info.obstacles.push_back(obs4);


  path_planner planner(v_info, obss_info);

  Eigen::Vector2d goal = {10, 10};
  std::stack<Node> waypoints;
  std::cout << "Calling library!" << std::endl;
  if (planner.ComputePath(goal, waypoints)) {
    //std::cout << "Found!" << std::endl;
    std::cout << "Found!" << std::endl;
    while (!waypoints.empty()) {
      Node wp = waypoints.top();
      waypoints.pop();
      //std::cout << "Pos: " << wp.position << std::endl;
      //std::cout << "Time: " << wp.time << std::endl;
    }
  } else {
    std::cout << "Not found." << std::endl;
  }


}
