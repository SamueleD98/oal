
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
  int scenario = 1;
  double count = 0;
  while (scenario == 1 && count < 1) {
    count++;

    std::cout << "----------------------------------------\n STARTING NEW PLAN " << std::endl;

    VehicleInfo v_info;
    Eigen::Vector2d goal;
    std::vector<Obstacle> obstacles;

    bb_data bb_dimension(20, 5,
                         1.6, 1.4,
                         2, 2,
                         1.5, 1.2,
                         1.5, 1.5,
                         0);

    //std::cout << "Which scenario?" << std::endl;
    //std::cin  >>  scenario;
    scenario = 1;
    //2, 3, 11, 10, 9, 8
    v_info.velocities = generateRange(1, 1, 0.1);

    /* TODO scenario
     * one where the vehicle should stand on but the ts is limited and so fast os cannot reach the front vxs
     *    ..does it return 'no path found' as it should?
     *
    */
    //flag = false;
    switch (scenario) {
      case 21: {
        //Overtaking and crossing situation on the high seas
        // https://www.advanced.ecolregs.com/index.php?option=com_k2&view=item&id=172:overtaking-and-crossing-situation-on-the-high-seas&Itemid=359&lang=en
        v_info.position = {10, 0};
        Obstacle obsB("B", {20, -10}, M_PI / 2, 1, M_PI / 2, bb_dimension);
        Obstacle obsC("C", {40, 40}, -M_PI * 5 / 6, 1, -M_PI * 5 / 6, bb_dimension);
        obstacles.push_back(obsB);
        obstacles.push_back(obsC);
        goal = {10, 50};
        break;

      }
      case 22:{
        //Overtaking and crossing situation on the high seas
        // https://www.advanced.ecolregs.com/index.php?option=com_k2&view=item&id=367:overtaking-and-crossing-situation-on-the-high-seas&Itemid=359&lang=en
        v_info.position = {10, 0};
        Obstacle obsB("B", {15, -10}, M_PI / 2, 1, M_PI / 2, bb_dimension);
        Obstacle obsC("C", {40, 30}, -M_PI, 1, -M_PI, bb_dimension);
        obstacles.push_back(obsB);
        obstacles.push_back(obsC);
        goal = {10, 50};
        break;
      }
      case 23:{
        //Overtaking and head-on situation on the high seas
        // https://www.advanced.ecolregs.com/index.php?option=com_k2&view=item&id=370:overtaking-and-head-on-situation-on-the-high-seas&Itemid=359&lang=en
        v_info.position = {10, 0};
        Obstacle obsB("B", {10, 20}, -M_PI / 2, 1, -M_PI / 2, bb_dimension);
        Obstacle obsC("C", {13.2, 25}, -M_PI / 2, 1, -M_PI / 2, bb_dimension);
        obstacles.push_back(obsB);
        obstacles.push_back(obsC);
        goal = {10, 20};
        break;
      }
      case 24:{
        //
        v_info.position = {10, 0};
        Obstacle obsB("B", {-5, 15}, 0, 1, 0, bb_dimension);
        Obstacle obsC("C", {-5, 10}, 0, 1, 0, bb_dimension, true);
        obstacles.push_back(obsB);
        obstacles.push_back(obsC);
        goal = {10, 20};
        break;
      }
      case 1: {// rand static or moving obstacles
        v_info.position = {10, 0};
        // Seed with a real random value, if available
        std::random_device r;
        // Choose a random mean between 1 and 6
        std::default_random_engine e1(r());
        std::uniform_real_distribution<double> pos_gen(-60, 60);
        std::uniform_real_distribution<double> speed_gen(0, 2);
        std::uniform_real_distribution<double> heading_gen(-M_PI, M_PI);
        std::uniform_real_distribution<double> vel_dir_gen(-M_PI/6, M_PI/6);
        for (auto i = 1; i < 10; i++) {
          double heading = heading_gen(e1);
          Obstacle obs(std::to_string(i), {pos_gen(e1), pos_gen(e1)}, heading, speed_gen(e1), heading+vel_dir_gen(e1), bb_dimension);
          obs.print();
          obstacles.push_back(obs);
        }
        goal = {10, 50};
        break;
      }

      case 2: {// head on WORKS (clear differences with/without Colregs)
        v_info.position = {10, 0};
        Obstacle obs("1", {10.5, 13}, -M_PI / 2, 1, -M_PI / 2, bb_dimension);
        obstacles.push_back(obs);
        goal = {10, 20};
        break;
      }
      case 3: {// TS crossing from right
        v_info.position = {10, 0};
        Obstacle obs("1", {24.4009, 10}, M_PI*6/7, 0.5, M_PI*6/7, bb_dimension);
        obstacles.push_back(obs);
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

      case 8: {// start in bb
        v_info.position = {10, 0};
        //Obstacle obs1("1", {13, 4.5}, 0, 3.12, 0, bb_dimension);
        Obstacle obs1("1", {-4, 0.5}, 0, 0.9, 0, bb_dimension);
        //Obstacle obs2("2", {10, 16}, -M_PI / 2, 0.9, -M_PI / 2, bb_dimension);
        //obstacles.push_back(obs1);
        obstacles.push_back(obs1);
        goal = {10, 40};
        break;
      }
      case 9: {// goal in bb
        v_info.position = {10, 0};
        Obstacle obs1("1", {10, 18}, M_PI / 2, 0, M_PI / 2, bb_dimension, true);
        obstacles.push_back(obs1);
        goal = {10, 20};
        break;
      }
      case 10: { // bb overlap
        v_info.position = {10, 0};
        Obstacle obs1("1", {9, 3.7}, 0, 0, 0, bb_dimension);
        obstacles.push_back(obs1);
        Obstacle obs2("2", {11, 4}, 0, 0, 0, bb_dimension);
        obstacles.push_back(obs1);
        goal = {10, 10};
        break;
      }
      case 11: { // goal surrounded
        v_info.position = {10, 0};
        Obstacle obs1("1", {10, 9}, 0, 0, 0, bb_dimension);
        obstacles.push_back(obs1);
        Obstacle obs2("2", {10, 11}, 0, 0, 0, bb_dimension);
        obstacles.push_back(obs1);
        Obstacle obs3("3", {11, 10}, 0, 0, 0, bb_dimension);
        obstacles.push_back(obs1);
        Obstacle obs4("4", {9, 10}, 0, 0, 0, bb_dimension);
        obstacles.push_back(obs1);
        obstacles.push_back(obs1);
        obstacles.push_back(obs2);
        obstacles.push_back(obs3);
        obstacles.push_back(obs4);
        goal = {10, 10};
        break;
      }
      default:
        break;
    }


    path_planner planner1(v_info, obstacles);
    path_planner planner2(v_info, obstacles);

    Path path1;
    std::cout << std::endl << "Colregs: false";
    if (planner1.ComputePath(goal, false, path1)) {
      std::cout << std::endl << "Found." << std::endl;
    } else {
      std::cout << std::endl << "Not found." << std::endl;
    }

    Path path2;
    std::cout << std::endl << "Colregs: true";
    if (planner2.ComputePath(goal, true, path2)) {
      if (planner2.CheckPath(v_info.position, path2)) {
        std::cout << "Checked!!" <<std::endl;
      }
    } else {
      std::cout << std::endl << "Not found." << std::endl;
    }

    // this after I'm done following the path (even a part of it)
    auto priorOvertakenVesselsList = path2.overtakingObsList;

    /*int s_count = 0;
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
      std::cout << "  speed_change: " << s_count<< std::endl;*/


  }




}
