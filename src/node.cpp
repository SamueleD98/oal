#include "data_structs/node.hpp"
#include "helper_functions.hpp"

void Node::UpdateCosts(const Eigen::Vector2d &goal) {
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
  Eigen::Vector2d node_pos((int) position[0] * 100, (int) position[1] * 100);
  for (const auto &node: set) {
    Eigen::Vector2d old_node_pos((int) node.position[0] * 100, (int) node.position[1] * 100);
    if (node_pos == old_node_pos && obs == node.obs && vx == node.vx) {
      return true;
    }
  }
  return false;
}

void Node::FindFisibilityVxs(Obstacle target_obs, std::vector<Vertex> &vxs_abs) {
  if (obs == target_obs.id) {
    // Set the adjacent vxs as visible
    if (vx == FR || vx == RL) {
      vxs_abs[1].isVisible = true;
      vxs_abs[2].isVisible = true;
    } else {
      vxs_abs[0].isVisible = true;
      vxs_abs[3].isVisible = true;
    }
  } else {
    // Find the angles between vectors from vehicle to each vxs and the vector from vehicle to the obs center,
    //  then the visible vxs are the ones with min/max angle + the ones closer to the vehicle than these two
    Eigen::Vector2d obs_position = ComputePosition(target_obs, time);
    Eigen::Vector2d obs_vh = obs_position - position;

    std::vector<double> thetas;  // angles wrt abs frame
    std::vector<Eigen::Vector2d> vxs_vh;

    for (Vertex &vx: vxs_abs) {
      Eigen::Vector2d vx_vh = vx.position - position; //if 0 there's a problem
      double sign = obs_vh.x() * vx_vh.y() - obs_vh.y() * vx_vh.x(); // same as z value of a cross product (and so its direction)
      double theta = sign / abs(sign) * acos(obs_vh.normalized().dot(vx_vh.normalized()));
      thetas.push_back(theta);
      vxs_vh.push_back(vx_vh);
    }
    auto max_v_ptr = std::max_element(thetas.begin(), thetas.end());
    auto max_v = std::distance(thetas.begin(), max_v_ptr);
    auto min_v_ptr = std::min_element(thetas.begin(), thetas.end());
    auto min_v = std::distance(thetas.begin(), min_v_ptr);
    // min/max angle vertexes are visible
    vxs_abs[min_v].isVisible = true;
    vxs_abs[max_v].isVisible = true;
    // are also visible vertexes which distance is smaller than those two
    double distance, min_v_distance, max_v_distance;
    min_v_distance = vxs_vh[min_v].norm();
    max_v_distance = vxs_vh[max_v].norm();
    for (size_t i = 0; i < vxs_vh.size(); ++i) {
      if (i == min_v || i == max_v) {
        continue;
      }
      distance = vxs_vh[i].norm();
      if (distance < min_v_distance && distance < max_v_distance) {
        vxs_abs[i].isVisible = true;
      }
    }
  }

}



