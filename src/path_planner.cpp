#include "oal/path_planner.hpp"



// Given some obstacle vertexes, find the intercept points
void path_planner::ComputeInterceptPoints(const Eigen::Vector2d &vehicle_position, const Obstacle &obstacle,
                                          std::vector<Vertex> &vertexes) const {
  for (Vertex &vertex: vertexes) {
    if (vertex.isVisible) {
      // algorithm described in
      // 'A Three-Layered Architecture for Real Time Path Planning and Obstacle Avoidance for Surveillance USVs Operating in Harbour Fields'
      Eigen::Vector2d vertex_vehicle = vertex.position - vehicle_position;
      double k = atan2(-vertex_vehicle[1], vertex_vehicle[0]); // error for (0,0)
      // NORM OF OBS_SPEED HAS TO BE EQUAL OR SMALLER THAN V_SPEED
      double theta = asin(obstacle.speed * sin(obstacle.heading + k) / v_info_.speed) - k;
      double t;
      if (abs(vertex_vehicle[0]) > 0) { // should it be something more than 0?
        t = vertex_vehicle[0] / (v_info_.speed * cos(theta) - obstacle.speed * cos(obstacle.heading));
      } else {
        t = vertex_vehicle[1] / (v_info_.speed * sin(theta) - obstacle.speed * sin(obstacle.heading));
      }

      Eigen::Vector2d intercept_point_position;
      intercept_point_position[0] = vehicle_position[0] + v_info_.speed * cos(theta) * t;
      intercept_point_position[1] = vehicle_position[1] + v_info_.speed * sin(theta) * t;

      vertex.ip_position = intercept_point_position;
      vertex.ip_time = t;
    }
  }
}

bool path_planner::CheckColreg(Node start, Node &goal){
  Eigen::Vector2d path = goal.position - start.position;
  double heading_v = atan2(path[1], path[0]); // error for (0,0)
  if(heading_v<0) heading_v+=2*M_PI;
  double theta = goal.obs_heading + heading_v;

  if(start.obs == goal.obs){
    return false;
  }

  //std::cout << theta << std::endl;

  if(theta > 15 *(M_PI/180) && theta < 112*(M_PI/180) && (goal.vx == RR || goal.vx == RL)){
    // crossing from right, should stand on
    return true;
  }

  if(theta > -(360-247.5) *(M_PI/180) && theta < -(360-345)*(M_PI/180) && (goal.vx == FR || goal.vx == FL)){
    // crossing from left, should give way
    return true;
  }

  if((theta > -(360-345) *(M_PI/180) && theta < 15*(M_PI/180)) && goal.vx == FR) {
    // head on, should be FL
    return true;
  }

  return false;
}

// Check if the path between start and goal collide with any obstacle
bool path_planner::CheckCollision(Node start, Node &goal, bool isFinalGoal) {
  Eigen::Vector3d start3d = {start.position[0], start.position[1], 0};
  Eigen::Vector3d goal3d = {goal.position[0], goal.position[1], 0};
  // vehicle path to check for collision
  Eigen::Vector3d path = goal3d - start3d;
  // time from start to goal
  goal3d[2] = path.norm() / v_info_.speed;
  path = goal3d - start3d;

  // plot
  plotCKFile_ << "---" << std::endl;
  plotCKFile_ << "Start_" << start3d(0) << "_" << start3d(1) << "_" << start3d(2) << std::endl;
  plotCKFile_ << "Goal_" << goal3d(0) << "_" << goal3d(1) << "_" << goal3d(2) << std::endl;

  for (Obstacle obs: obss_info_.obstacles) {

    // if the goal obstacle is this one, then do not check, the algorithm already aims for the visible vxs
    // (this assumption does not hold with overlapping bounding boxes)
    /*if (obs.id == goal.obs) {
      continue;
    } moved down to plot stuff*/

    // Compute obstacle direction in x-y-t
    Eigen::Vector3d bb_direction;
    bb_direction[0] = obs.speed * cos(obs.heading);
    bb_direction[1] = obs.speed * sin(obs.heading);
    bb_direction[2] = 1;
    //bb_direction.normalize(); no, because multiplied by t* it returns the position at time t*

    // Find vertexes in abs frame
    std::vector<Vertex> vxs_abs;
    ComputeAbsVertexes(obs, start.time, vxs_abs);

    plotCKFile_ << "Obs_" << obs.id << std::endl;

    // Uncomment to plot
    if(start.obs == "1" && start.obs == goal.obs) {
      plotCKFile_ << "PlotIt" << std::endl;
    }

    plotCKFile_ << "Direction_" << bb_direction(0) << "_" << bb_direction(1) << "_" << bb_direction(2) << std::endl;
    for (Vertex vx: vxs_abs) {
      plotCKFile_ << "Vx_" << vx.position[0] << "_" << vx.position[1] << std::endl;
    }
    plotCKFile_ << "-" << std::endl;

    if (obs.id == goal.obs) {
      continue;
    }
    
    // for each of the bb 4 sides
    for (int side_idx = 0; side_idx < 4; side_idx++) {
      int vx_idx1, vx_idx2;
      switch (side_idx) {
        case 0:
          vx_idx1 = 0;
          vx_idx2 = 2;
          break;
        case 1:
          vx_idx1 = 0;
          vx_idx2 = 1;
          break;
        case 2:
          vx_idx1 = 3;
          vx_idx2 = 2;
          break;
        case 3:
          vx_idx1 = 3;
          vx_idx2 = 1;
          break;
        default:
          continue;
      }

      // if the path starts from the current obs, do not check the sides of the departing vx
      if (obs.id == start.obs && (start.vx == vx_idx1 || start.vx == vx_idx2)) {
        continue;
      }
       /*blindly jump the check here can cause collision when the goal is on another bb side that is then not incident with the path
       resulting in false negative outcome and so a collision because the current side is traversed by own ship
       */

      Eigen::Vector3d bb_vector1(vxs_abs[vx_idx1].position[0], vxs_abs[vx_idx1].position[1], 0);
      Eigen::Vector3d bb_vector2(vxs_abs[vx_idx2].position[0], vxs_abs[vx_idx2].position[1], 0);

      // Get the plane normal
      Eigen::Vector3d side = bb_vector1 - bb_vector2;
      Eigen::Vector3d planeNormal = bb_direction.cross(side);
      planeNormal.normalize();

      // if the plane and the path are //
      if (planeNormal.dot(path) == 0) continue; //do we need tolerance here?

      // Compute position along the path where the collision happens (from http://paulbourke.net/geometry/pointlineplane/)
      Eigen::Vector3d startToOne = bb_vector1 - start3d;
      double c_pos = planeNormal.dot(startToOne) / planeNormal.dot(path);

      // collision_pos has to be between 0 and 1 (start and goal) to be an actual problem
      if (c_pos >= 0 && c_pos <= 1) { // intersect_point between start and goal
        // what about u==1 ? means the collision happens when reaching the vertex,
        //  that makes sense when approaching the obstacle I'm aiming to

        // Get collision time and point
        Eigen::Vector3d doable_path = path * c_pos; // Part of the path prior the collision
        doable_path[2] = 0;
        double collision_time = doable_path.norm() / v_info_.speed; // When the collision happens
        doable_path[2] = collision_time;
        Eigen::Vector3d collision_point = start3d + doable_path; // Where the collision happens

        // Get vertexes at time t*
        Eigen::Vector3d vertex1_position = bb_vector1 + bb_direction * collision_time;
        Eigen::Vector3d vertex2_position = bb_vector2 + bb_direction * collision_time;

        // Check if point is inside those two vertexes
        Eigen::Vector3d P1 = collision_point - vertex1_position;
        Eigen::Vector3d P2 = collision_point - vertex2_position;


        if (P1.dot(P2) <= 0) {
          if (isFinalGoal) {
            // so there's a collision, and it is the final goal
            // then check if the goal, at goal time, is in this obstacle bb
            // if it is, check for a closer goal, if it is not then at least one more waypoint has to be computed
            // and so return false

            // Find goal wrt obs frame at goal time
            Eigen::Vector2d goal_obs = goal.position - ComputePosition(obs, goal3d[2]);
            Eigen::Rotation2D<double> rotation(obs.heading);
            Eigen::Vector2d bodyObs_goal = rotation.inverse() * goal_obs;

            if (abs(bodyObs_goal.x()) <= abs(obs.vxs[1].position.x()) &&
                abs(bodyObs_goal.x()) <= abs(obs.vxs[0].position.x()) &&
                abs(bodyObs_goal.y()) <= abs(obs.vxs[3].position.y()) &&
                abs(bodyObs_goal.y()) <= abs(obs.vxs[1].position.y())) {

              Node newgoal;
              Eigen::Vector2d collision_point_2d(collision_point[0], collision_point[1]);
              newgoal.position = collision_point_2d;
              newgoal.obs = obs.id;

              newgoal.time = start.time + collision_time;

              // Being the boolean to false, there can't be more than one reiteration:
              // if there's no collision then update the goal, otherwise keep looking

              if (CheckCollision(start, newgoal, false)) {
                //update goal
                goal = newgoal;
                plotCKFile_ << "Good" << std::endl;
                return true;
              }
            }
          }

          return false;

        }
      }
    }
  }
  plotCKFile_ << "Good" << std::endl;
  return true;
}

// Compute the path to reach the goal
bool path_planner::ComputePath(const Eigen::Vector2d &goal_position, std::stack<Node> &waypoints) {
  // Plot stuff
  plotWpsFile_ << "Start_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
  plotWpsFile_ << "Goal_" << goal_position(0) << "_" << goal_position(1) << std::endl;

  bool found = false;
  // set of viable nodes, ordered by total cost (ascending)
  std::multiset<Node> nodes_set;

  std::multiset<Node> trash_set;

  // The planner starts from the vehicle position with time=0
  Node root;
  root.position = v_info_.position;
  root.time = 0;
  UpdateCosts(root, goal_position, v_info_.speed);
  nodes_set.insert(root);

  Node goal;
  goal.position = goal_position;

  // Set the bounding box dimension based on the vehicle distance from the obstacles
  for (Obstacle &obs: obss_info_.obstacles) {
    obs.ComputeLocalVertexes(v_info_.position);
  }

  Node current;
  // Run until every node is studied or a path is found
  while (!nodes_set.empty()) {
    current = *nodes_set.begin();
    nodes_set.erase(nodes_set.begin());

    if(current.time >10){
      bool tr = true;
    }

    // if goal in a bb, compute closer goal
    // if closer_g is doable,

    if (CheckCollision(current, goal, true)) {
      found = true;
      break;
    }

    for (Obstacle &obs: obss_info_.obstacles) {
      // Find vertexes in abs frame
      std::vector<Vertex> vxs_abs;
      ComputeAbsVertexes(obs, current.time, vxs_abs);

      // Find vxs visibility
      if (current.obs == obs.id) {
        // Set the adjacent vxs as visible
        if (current.vx == FR || current.vx == RL){
          vxs_abs[1].isVisible = true;
          vxs_abs[2].isVisible = true;
        }else{
          vxs_abs[0].isVisible = true;
          vxs_abs[3].isVisible = true;
        }
      } else {
        FindVisibility(current.position, vxs_abs);
      }
      // Compute when every visible vertex is intercepted
      ComputeInterceptPoints(current.position, obs, vxs_abs);
      // Check collision for every one of them
      for (const Vertex &vx: vxs_abs) {
        if (vx.isVisible) {



          // Create new Node (still to check for collision)
          Node newnode;
          newnode.position = vx.ip_position;
          newnode.obs = obs.id;
          newnode.obs_heading = obs.heading;
          newnode.vx = vx.id;
          newnode.time = current.time + vx.ip_time;
          std::cout << "." << std::endl;
          if (CheckColreg(current, newnode)){
            continue;
          }
          if (CheckCollision(current, newnode, false)) {

            if(AlreadyExists(newnode, nodes_set)){
              trash_set.insert(newnode);
              continue;
            }

            if(AlreadyExists(newnode, trash_set)){
              continue;
            }

            // Save new node
            UpdateCosts(newnode, goal_position, v_info_.speed);
            std::shared_ptr<Node> parent = std::make_shared<Node>(current);
            newnode.parent = parent;
            nodes_set.insert(newnode);
          }
        }
      }
    }
  }


  if (found) {
    // Build path from goal to initial position
    Node wp;
    Node *it = &current;

    // Plot stuff
    plotWpsFile_ << "Time_" << current.costTotal << std::endl;
    plotWpsFile_ << "Waypoint_" << goal.position(0) << "_" << goal.position(1) << std::endl;
    for (Obstacle &obs: obss_info_.obstacles) {
      plotWpsFile_ << "Obs_" << obs.id << std::endl;
      Eigen::Vector2d pose = ComputePosition(obs, current.costTotal);
      plotWpsFile_ << "Pose_" << pose(0) << "_" << pose(1) << std::endl;
      plotWpsFile_ << "Heading_" << obs.heading << std::endl;
      plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
      plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
      plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
      plotWpsFile_ << "Max_" << obs.max_bb_ratio << std::endl;
      plotWpsFile_ << "-" << std::endl;
    }

    while (it->parent != nullptr) {
      wp.position = it->position;
      wp.time = it->time;
      waypoints.push(wp);
      it = &*it->parent;

      //  Plot stuff
      plotWpsFile_ << "Time_" << wp.time << std::endl;
      plotWpsFile_ << "Waypoint_" << wp.position(0) << "_" << wp.position(1) << std::endl;
      for (Obstacle &obs: obss_info_.obstacles) {
        plotWpsFile_ << "Obs_" << obs.id << std::endl;
        Eigen::Vector2d pose = ComputePosition(obs, wp.time);
        plotWpsFile_ << "Pose_" << pose(0) << "_" << pose(1) << std::endl;
        plotWpsFile_ << "Heading_" << obs.heading << std::endl;
        plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
        plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
        plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
        plotWpsFile_ << "Max_" << obs.max_bb_ratio << std::endl;
        plotWpsFile_ << "-" << std::endl;
      }
    }

    // Plot stuff
    plotWpsFile_ << "Time_" << "0" << std::endl;
    plotWpsFile_ << "Waypoint_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
    for (Obstacle &obs: obss_info_.obstacles) {
      plotWpsFile_ << "Obs_" << obs.id << std::endl;
      Eigen::Vector2d pose = ComputePosition(obs, it->time);
      plotWpsFile_ << "Pose_" << pose(0) << "_" << pose(1) << std::endl;
      plotWpsFile_ << "Heading_" << obs.heading << std::endl;
      plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
      plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
      plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
      plotWpsFile_ << "Max_" << obs.max_bb_ratio << std::endl;
      plotWpsFile_ << "-" << std::endl;
    }
    return true;
  }
  return false;
}

void path_planner::FindVisibility(const Eigen::Vector2d &vh_pos, std::vector<Vertex> &vxs_abs) {
  // The function finds the angles of the vxs wrt the vehicle,
  //  then the visible ones are the ones with min/max angle + the ones closer to the vehicle than these two
  std::vector<double> thetas;  // angles wrt abs frame
  std::vector<Eigen::Vector2d> vxs_vh;

  for (Vertex &vx: vxs_abs) {
    Eigen::Vector2d vx_vehicle = vx.position - vh_pos; //if 0 there's a problem
    double theta;
    theta = std::atan2(vx_vehicle[1], vx_vehicle[0]);
    if (theta<0){
      theta = theta + 2*M_PI;
    }
    thetas.push_back(theta);
    vxs_vh.push_back(vx_vehicle);
  }
  // not good, check the kinda difference of the angles!!!!!!!!!!ppppppppppppppp
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
