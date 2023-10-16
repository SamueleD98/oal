#include "data_structs/node.hpp"

void Node::UpdateCosts(const Eigen::Vector2d &goal, double vh_speed) {
  costToReach = time; //if the cost is the time to reach the target
  Eigen::Vector2d dist_to_goal = goal - position;
  costToGoal = dist_to_goal.norm() / vh_speed;
  costTotal = costToReach + costToGoal;
}

void Node::GetCloser(const std::shared_ptr<std::vector<Node>>& nodes_list, Node& closer) const{
  if(nodes_list->empty()){
    closer = *this;
  }else{
    closer = nodes_list->at(0);
    Eigen::Vector2d dist_v = closer.position - position;
    double min_distance = dist_v.norm();
    for (const auto& node : *nodes_list) {
      dist_v = node.position - position;
      double dist = dist_v.norm();
      if (dist < min_distance) {
        closer = node;
        min_distance = dist;
      }
    }
  }
}

bool Node::IsIn(std::multiset<Node> &set) {
  Eigen::Vector2d new_node_pos((int) position[0] * 100, (int) position[1] * 100);
  for (const auto &node: set) {
    Eigen::Vector2d old_node_pos((int) node.position[0] * 100, (int) node.position[1] * 100);
    if (new_node_pos == old_node_pos && obs == node.obs && vx == node.vx) {
      return true;
    }
  }
  return false;
}

