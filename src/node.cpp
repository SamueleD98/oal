#include "oal/data_structs/node.hpp"
#include "oal/helper_functions.hpp"

void Node::UpdateCosts(const Eigen::Vector2d &goal) {
  costToReach = time; //if the cost is the time to reach the target
  Eigen::Vector2d dist_to_goal = goal - position;
  costToGoal = dist_to_goal.norm() / vh_speed;
  costTotal = costToReach + costToGoal;
}

void Node::GetCloser(const std::shared_ptr<std::vector<Node>> &nodes_list, Node &closer) const {
  if (nodes_list->empty()) {
    closer = *this;
  } else {
    closer = nodes_list->at(0);
    Eigen::Vector2d dist_v = closer.position - position;
    double min_distance = dist_v.norm();
    for (const auto &node: *nodes_list) {
      dist_v = node.position - position;
      double dist = dist_v.norm();
      if (dist < min_distance) {
        closer = node;
        min_distance = dist;
      }
    }
  }
}

bool Node::IsUnique(Node other) {
  while (other.parent != nullptr) {
    if (/*node_pos == other_pos && */obs_ptr.get() == other.obs_ptr.get() && vx == other.vx) {
      return false;
    }
    other = *other.parent;
  }
  return true;
}

void Node::FindVisibilityVxs(Obstacle target_obs, std::vector<Vertex> &vxs_abs) {
  if (obs_ptr != nullptr) {
    if (obs_ptr->id == target_obs.id) {
      if (vx == NA){
        // OS in TS bb, visible are the exit ones
        std::vector<vx_id> allowedVxs;
        FindExitVxs(target_obs, allowedVxs);
        vxs_abs[allowedVxs[0]].isVisible = true;
        vxs_abs[allowedVxs[1]].isVisible = true;
      }
      // Set the adjacent vxs as visible
      if (vx == FR || vx == RL) {
        vxs_abs[1].isVisible = true;
        vxs_abs[2].isVisible = true;
      } else {
        vxs_abs[0].isVisible = true;
        vxs_abs[3].isVisible = true;
      }
      return;
    }
  }

  // Find the angles between vectors from vehicle to each vxs and the vector from vehicle to the obs center,
  //  then the visible vxs are the ones with min/max angle + the ones closer to the vehicle than these two
  Eigen::Vector2d obs_position = ComputePosition(target_obs, time);
  Eigen::Vector2d obs_vh = obs_position - position;

  std::vector<double> thetas;  // angles wrt abs frame
  std::vector<Eigen::Vector2d> vxs_vh;

  for (Vertex &vert: vxs_abs) {
    Eigen::Vector2d vx_vh = vert.position - position; //if 0 there's a problem
    double sign = obs_vh.x() * vx_vh.y() -
                  obs_vh.y() * vx_vh.x(); // same as z value of a cross product (and so its direction)
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

bool Node::IsInSet(std::multiset<Node> &set) const {
  for (const auto &node: set) {
    if (node == *this) {
      node.print();
      this->print();
      std::cout << "___" << std::endl;
      return true;
    }
  }
  return false;
}

void Node::FindExitVxs(const Obstacle &obs, std::vector<vx_id> &allowedVxs) const {
  Eigen::Vector2d bodyObs_e = GetProjectionInObsFrame(position, obs, 0.0);

  bool IsLeftOfDiag1 = (bodyObs_e.y() >=
                        obs.vxs[1].position.y() / obs.vxs[1].position.x() *
                        bodyObs_e.x()); // Being left of diag FL-RR
  bool IsLeftOfDiag2 = (bodyObs_e.y() >=
                        obs.vxs[0].position.y() / obs.vxs[0].position.x() *
                        bodyObs_e.x()); // Being left of diag FR-LL

  if (IsLeftOfDiag1 && IsLeftOfDiag2) {
    // USV in port side of bb
    //  go for FL or RL
    allowedVxs.push_back(FL);
    allowedVxs.push_back(RL);
  } else {
    if (IsLeftOfDiag1) {
      // USV in stern side of bb
      //  go for RL or RR
      allowedVxs.push_back(RL);
      allowedVxs.push_back(RR);
    } else {
      if (IsLeftOfDiag2) {
        // USV in bow side of bb
        //  go for FR or FL
        allowedVxs.push_back(FR);
        allowedVxs.push_back(FL);
      } else {
        // USV in starboard side of bb
        //  go for FR or RR
        allowedVxs.push_back(FR);
        allowedVxs.push_back(RR);
      }
    }
  }
}




