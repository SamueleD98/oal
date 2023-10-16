#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <stack>
#include "obstacle.hpp"
#include "node.hpp"

//#include "vertex.hpp"

struct VehicleInfo {
    Eigen::Vector2d position;
    double speed;
};

struct ObstaclesInfo {
  std::vector<std::shared_ptr<Obstacle>> obstacles;
};

struct Path {
    std::stack<Node> waypoints;
    std::vector<std::string> overtakingObsList;
};

#endif
