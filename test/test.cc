
#include <random>
#include "path_planner.hpp"

std::vector<double> generateRange(double start, double end, double step) {
  std::vector<double> result;
  for (double i = start; i <= end; i += step) {
    double num = std::round(i * 100) / 100;
    if (num != 0) result.push_back(num);
  }
  return result;
}

int main(int, char **) {

  VehicleInfo v_info;
  Eigen::Vector2d goal;
  std::vector<Obstacle> obstacles;

  int scenario;

  //std::cout << "Which scenario?" << std::endl;
  //std::cin  >>  scenario;
  scenario = 3;

  /* TODO scenario
   * one where the vehicle should stand on but the ts is limited and so fast os cannot reach the front vxs
   *    ..does it return 'no path found' as it should?
   *
  */

  switch (scenario) {
    case 1: {// rand static or moving obstacles
      v_info.position = {10, 0};

      // Seed with a real random value, if available
      std::random_device r;
      // Choose a random mean between 1 and 6
      std::default_random_engine e1(r());
      std::uniform_real_distribution<double> pos_gen(-20, 20);
      std::uniform_real_distribution<double> speed_gen(0, 2);
      std::uniform_real_distribution<double> heading_gen(-M_PI, M_PI);

      for (auto i = 1; i < 15; i++) {
        Obstacle obs = Obstacle(std::to_string(i), {pos_gen(e1), pos_gen(e1)}, heading_gen(e1), speed_gen(e1), 2, 0.5, 3.5,
                                6, 3.5, 3);
        obs.print();
        obstacles.push_back(obs);
      }

      goal = {10, 20};
      break;
    }

    case 2: {// head on WORKS (clear differences with/without Colregs)
      v_info.position = {10, 0};
      Obstacle obs1_1 = Obstacle("1", {10.5, 13}, -M_PI / 2, 0.5, 2, 0.5, 3.2, 9, 3.2, 3);
//      Obstacle obs1_2 = Obstacle("1", {10.5, 13}, 0, 0, 2, 0.5, 3.2, 9, 3.2, 3);

//      obss_info.obstacles.push_back(std::make_shared<Obstacle>(obs1_1));
      obstacles.push_back(obs1_1);
      goal = {10, 20};
      break;
    }
    case 3: {// TS crossing from right
      v_info.position = {10, 0};
      Obstacle obs3_1 = Obstacle("1", {24.4009, 10}, M_PI*6/7, 0.5, 2, 0.5, 3.2, 9, 3.2, 3);
      obstacles.push_back(obs3_1);
      goal = {10, 20};
      break;
    }
    case 4: {// crossing left
      v_info.position = {10, 0};
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
      Obstacle obs1_1 = Obstacle("1", {10, 2.5}, M_PI / 2, 3.12, 2, 1, 3.2, 4, 4, 1.1);
      Obstacle obs1_4 = Obstacle("1", {10, 2}, -M_PI / 2, 1, 2, 1, 3.2, 4, 4, 1.1);
      obstacles.push_back(obs1_4);

      goal = {10, 20};
      break;
    }
    case 9: {// goal in bb
      v_info.position = {10, 0};
      Obstacle obs1_1 = Obstacle("1", {10, 18}, M_PI / 2, 0, 2, 0.5, 3.2, 9, 3.2, 3, true);

      obstacles.push_back(obs1_1);

      goal = {10, 20};
      break;
    }
    case 10: { // bb overlap
      v_info.position = {10, 0};
      Obstacle obs1 = Obstacle("1", {9, 3.7}, 0, 0, 1, 0.5, 2, 3, 3, 1);
      Obstacle obs2 = Obstacle("2", {11, 4}, 0, 0, 0.5, 0.5, 2, 3, 3, 1);

      //Obstacle obs1_2 = Obstacle("2", {10.5, 3.5}, M_PI, 0, 1, 0.5, 4, 1.6);
      //Obstacle obs1_3 = Obstacle("3", {9, 4}, M_PI, 0, 0.5, 0.5, 4, 1.6);
      obstacles.push_back(obs1);
      obstacles.push_back(obs2);
      //obss_info.obstacles.push_back(obs1_3);
      goal = {10, 10};
      break;
    }

    case 11: { // goal surrounded
      v_info.position = {10, 0};
      Obstacle obs1_1 = Obstacle("1", {10, 9}, 0, 0, 2, 0.5, 2, 2, 2, 1);
      Obstacle obs1_2 = Obstacle("2", {10, 11}, 0, 0, 2, 0.5, 2, 2, 2, 1);
      Obstacle obs1_3 = Obstacle("3", {11, 10}, 0, 0, 2, 0.5, 2, 2, 2, 1);
      Obstacle obs1_4 = Obstacle("4", {9, 10}, 0, 0, 2, 0.5, 2, 2, 2, 1);


      obstacles.push_back(obs1_1);
      obstacles.push_back(obs1_2);
      obstacles.push_back(obs1_3);
      obstacles.push_back(obs1_4);
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


  v_info.velocities = generateRange(1, 1, 0.1);
  path_planner planner1(v_info, obstacles);
  path_planner planner2(v_info, obstacles);

  Path path1;
  std::cout <<std::endl << "Colregs: false";
  if (planner1.ComputePath(goal, false, path1)) {
    int s_count = -1;
    double s_value = 0;
    Node wp;
    while (!path1.empty()) {
      wp = path1.top();
      path1.pop();
      if (wp.vh_speed != s_value) {
        s_count++;
        s_value = wp.vh_speed;
      }
    }
    std::cout << "  time: " << wp.time;
    if(wp.time<=0) throw std::invalid_argument("wtf");
    std::cout << "  speed_change: " << s_count<< std::endl;
    /*if (planner.CheckPath(v_info.position, 0, path1)) {
      std::cout << "Checked!!" << std::endl;
    }*/
  } else {
    std::cout <<std::endl << "Not found." << std::endl;
  }

  Path path2;
  std::cout <<std::endl << "Colregs: true";
  if (planner2.ComputePath(goal, true, path2)) {
    int s_count = -1;
    double s_value = 0;
    Node wp;
    while (!path2.empty()) {
      wp = path2.top();
      path2.pop();
      if (wp.vh_speed != s_value) {
        s_count++;
        s_value = wp.vh_speed;
      }
    }
    std::cout << "  Time: " << wp.time;
    if(wp.time<=0) throw std::invalid_argument("wtf");
    std::cout << "  speed_change: " << s_count<< std::endl;
    if (planner2.CheckPath(v_info.position, 0, path2)) {
      //std::cout << "Checked!!" << std::endl;
    }
  } else {
    std::cout <<std::endl<< "Not found." << std::endl;
  }

  // this after I'm done following the path (even a part of it)
  auto priorOvertakenVesselsList = path2.overtakingObsList;


}
