#ifndef OAL_NODE_HPP
#define OAL_NODE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include "misc.hpp"

class Node {
public:
    Eigen::Vector2d position; //vehicle position
    // time and costToReach are the same when the cost==time to reach the target
    double time = -1;  // time instant
    double costToReach = -1; //cost to reach the Node
    double costToGoal = -1; //estimated cost to reach Goal
    double costTotal = -1; //g+h total cost
    std::string obs;
    double obs_heading = -4;
    vx_id vx;
    std::shared_ptr<Node> parent = nullptr;
    std::vector<vx_id> colregsLimitedVxs;

    // Used to order nodes in set according to total cost to reach the goal
    bool operator<(const Node &other) const {
      return costTotal < other.costTotal;
    }

    void UpdateCosts(const Eigen::Vector2d &goal, double speed);
};


#endif //OAL_NODE_HPP
