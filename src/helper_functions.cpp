#include "helper_functions.hpp"

void FindAbsVxs(Obstacle &obs, double time, std::vector<Vertex> &vxs_abs) {
  Eigen::Vector2d current_obs_position = ComputePosition(obs, time);
  for (const Vertex &vx: obs.vxs) {
    Vertex vx_abs;
    vx_abs.id = vx.id;
    Eigen::Rotation2D<double> rotation(obs.heading);
    vx_abs.position = current_obs_position + rotation * vx.position;
    vxs_abs.push_back(vx_abs);
  }
}

bool IsIn(Node new_node, std::multiset<Node> &set) {
  Eigen::Vector2d new_node_pos((int) new_node.position[0] * 100, (int) new_node.position[1] * 100);
  for (const auto &node: set) {
    Eigen::Vector2d old_node_pos((int) node.position[0] * 100, (int) node.position[1] * 100);
    if (new_node_pos == old_node_pos && new_node.obs == node.obs && new_node.vx == node.vx) {
      return true;
    }
  }
  return false;
}


