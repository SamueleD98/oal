
#include "path_planner.hpp"

int main(int, char **) {

  VehicleInfo v_info;
  Eigen::Vector2d goal;
  ObstaclesInfo obss_info;

  bool colregs = false;
  int scenario;

  std::cout << "Which scenario?" << std::endl;
  //std::cin  >>  scenario;
  scenario = 8;

  switch (scenario) {
    case 1: {// static or moving obstacles, no colregs
      v_info.position = {10, 0};
      v_info.speed = 2;
      Obstacle obs1_1 = Obstacle("1", {10, 5}, M_PI / 2, 0.5, 2, 0.5, 3.2, 9, 3.2, 3, true);
      Obstacle obs1_2 = Obstacle("2", {9, 10}, M_PI, 0.5, 2, 0.5, 3.2, 5, 3.2, 3);
      Obstacle obs1_3 = Obstacle("3", {8, 14}, M_PI * 1 / 4, 0.5, 2, 0.5, 3.2, 9, 3.2, 3);
      obss_info.obstacles.push_back(obs1_1);
      obss_info.obstacles.push_back(obs1_2);
      obss_info.obstacles.push_back(obs1_3);
      goal = {10, 20};
      break;
    }
    case 2: {// head on
      v_info.position = {10, 0};
      v_info.speed = 2;
      //Obstacle obs2_1 = Obstacle("1", {10.2, 10}, M_PI*3/2, 1, 0.5, 0.5, 2, 1.5);
      //obss_info.obstacles.push_back(obs2_1);
      goal = {10, 10};
      break;
    }
    case 3: {// TS crossing from right
      v_info.position = {10, 0};
      v_info.speed = 2;
      //Obstacle obs3_1 = Obstacle("1", {12.5, 4}, M_PI, 1, 0.5, 0.5, 2, 1.5);
      //obss_info.obstacles.push_back(obs3_1);
      goal = {10, 10};
      break;
    }
    case 4: {// crossing left
      v_info.position = {10, 0};
      v_info.speed = 2;
      //Obstacle obs4_1 = Obstacle("1", {8.5, 4}, 0, 1, 0.5, 0.5, 2, 1.5);
      //obss_info.obstacles.push_back(obs4_1);
      goal = {10, 10};
      break;
    }


    case 5: // overtake
      break;
    case 6: // overtaken
      break;
    case 7: // multi-encounter
      break;
    case 8: {// start in bb
      v_info.position = {10, 0};
      v_info.speed = 2;
      Obstacle obs1_1 = Obstacle("1", {10, 2.5}, M_PI / 2, 3.12, 2, 1, 3.2, 4, 4, 1.1);
      Obstacle obs1_4 = Obstacle("1", {10, 2.5}, -M_PI / 2, 1.9, 2, 1, 3.2, 4, 4, 1.1);

      Obstacle obs1_2 = Obstacle("2", {9, 10}, M_PI, 0.5, 2, 0.5, 3.2, 5, 3.2, 3);
      Obstacle obs1_3 = Obstacle("3", {8, 14}, M_PI * 1 / 4, 0.5, 2, 0.5, 3.2, 9, 3.2, 3);
      obss_info.obstacles.push_back(obs1_4);

      goal = {10, 20};
      break;
    }
    case 9: // goal in bb
      break;
    case 10: { // bb overlap
      v_info.position = {10, 0};
      v_info.speed = 2;
      Obstacle obs1 = Obstacle("1", {9, 3.7}, 0, 0, 1, 0.5, 2, 3, 3, 1);
      Obstacle obs2 = Obstacle("2", {11, 4}, 0, 0, 0.5, 0.5, 2, 3, 3, 1);

      //Obstacle obs1_2 = Obstacle("2", {10.5, 3.5}, M_PI, 0, 1, 0.5, 4, 1.6);
      //Obstacle obs1_3 = Obstacle("3", {9, 4}, M_PI, 0, 0.5, 0.5, 4, 1.6);
      obss_info.obstacles.push_back(obs1);
      obss_info.obstacles.push_back(obs2);
      //obss_info.obstacles.push_back(obs1_3);
      goal = {10, 10};
      break;
    }
    default:
      break;
  }


  // Obstacle co  nstructor arguments: string id, Eigen::Vector2d position, double heading, double speed, double dim_x, double dim_y, double max_bb_ratio, double safety_bb_ratio
  /*Obstacle obs1 = Obstacle("1", {10, 4}, 3.14 / 2, 0.8, 0.5, 0.5, 2, 1.6);
  Obstacle obs2 = Obstacle("2", {9, 4.09}, 0, 0.5, 0.4, 0.4, 2, 1.6);
  Obstacle obs3 = Obstacle("3", {11, 0.5}, 3.14 * 3 / 4, 1.8, 0.2, 0.2, 2, 1.6);
  Obstacle obs4 = Obstacle("4", {5, 0.5}, 3.14 / 3, 2, 0.2, 0.2, 2, 1.6);
*/

  /* overlapping bb
  Obstacle obs1 = Obstacle("1", {10, 1}, 0, 0, 0.5, 0.5, 6, 5);
  Obstacle obs2 = Obstacle("2", {10, 9}, 0, 0, 1, 1, 2, 1.6);
  Obstacle obs3 = Obstacle("3", {11, 10}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs4 = Obstacle("4", {9, 10}, 0, 0, 0.5, 0.5, 2, 1.6);


  /*
  // surround the goal
  Obstacle obs1 = Obstacle("1", {10, 9}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs2 = Obstacle("2", {10, 11}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs3 = Obstacle("3", {11, 10}, 0, 0, 0.5, 0.5, 2, 1.6);
  Obstacle obs4 = Obstacle("4", {9, 10}, 0, 0, 0.5, 0.5, 2, 1.6);
   */



  path_planner planner(v_info, obss_info);
  std::stack<Node> waypoints;
  std::cout << "Calling library!" << std::endl;
  if (planner.ComputePath(goal, colregs, waypoints)) {
    //std::cout << "Found!" << std::endl;
    std::cout << "Found!" << std::endl;
    if(false){
      while (!waypoints.empty()) {
        Node wp = waypoints.top();
        waypoints.pop();

          std::cout << "Pos: " << wp.position.x() << "_" << wp.position.y() << std::endl;
          std::cout << "Time: " << wp.time << std::endl;
      }
    }else{
      if(planner.CheckPath(v_info.position, 0, waypoints)){
        std::cout << "checked!!" << std::endl;
      }
    }
  } else {
    std::cout << "Not found." << std::endl;
  }

}
