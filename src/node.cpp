#include "data_structs/node.hpp"

void Node::UpdateCosts(const Eigen::Vector2d &goal, double speed) {
  costToReach = time; //if the cost is the time to reach the target
  Eigen::Vector2d dist_to_goal = goal - position;
  costToGoal = dist_to_goal.norm() / speed;
  costTotal = costToReach + costToGoal;
}

