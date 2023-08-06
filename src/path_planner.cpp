#include "oal/path_planner.hpp"

// Given some obstacle vertexes, find the intercept points
void path_planner::ComputeInterceptPoints(const Eigen::Vector2d& vehicle_position, const Obstacle& obstacle,  std::vector<Vertex>& vertexes){
  for (Vertex& vertex : vertexes){
    if(vertex.isVisible){
      // algorithm described in
      // 'A Three-Layered Architecture for Real Time Path Planning and Obstacle Avoidance for Surveillance USVs Operating in Harbour Fields'
      Eigen::Vector2d vertex_vehicle = vertex.position - vehicle_position;
      double k = atan2(-vertex_vehicle[1], vertex_vehicle[0]); // error for (0,0)
      // NORM OF OBS_SPEED HAS TO BE EQUAL OR SMALLER THAN V_SPEED
      double theta = asin(obstacle.speed_ / v_info_.speed * sin(obstacle.heading_ + k)) - k;
      double t;
      if(abs(vertex_vehicle[0]) > 0){ // should it be something more than 0?
        t = vertex_vehicle[0] / ( v_info_.speed * cos(theta) - obstacle.speed_ * cos(obstacle.heading_));
      }else{
        t = vertex_vehicle[1] / ( v_info_.speed * sin(theta) - obstacle.speed_ * sin(obstacle.heading_));
      }

      Eigen::Vector2d intercept_point_position;
      intercept_point_position[0] = vehicle_position[0]+v_info_.speed*cos(theta)*t;
      intercept_point_position[1] = vehicle_position[1]+v_info_.speed*sin(theta)*t;
      if(intercept_point_position[1] <1){
        int afsac=1;
      }

      vertex.intercept_point.position = intercept_point_position;
      vertex.intercept_point.time = t;
    }
  }
}

// Check if the path between start and goal collide with any obstacle
bool path_planner::CheckCollision(Node start, Node goal){
  Eigen::Vector3d start3d = {start.position[0], start.position[1], 0};
  Eigen::Vector3d goal3d = {goal.position[0], goal.position[1], 0};
  // vehicle path to check for collision
  Eigen::Vector3d path = goal3d - start3d;
  // time from start to goal
  goal3d[2] = path.norm()/v_info_.speed;
  path = goal3d - start3d;

  // plot
  if(plotCKFile_.is_open()){
    plotCKFile_.close();
  }
  plotCKFile_.open("CKlog.txt",std::ofstream::app);
  plotCKFile_ << "---" << std::endl;
  plotCKFile_ << "Start_" << start3d(0) << "_" << start3d(1) << "_" << start3d(2) << std::endl;
  plotCKFile_ << "Goal_" << goal3d(0) << "_" << goal3d(1) << "_" << goal3d(2) << std::endl;

  for (Obstacle obs : obss_info_.obstacles){
    //std::vector<std::vector<int>> bb_vectors_couples(4); // 4 couples of vertexes to check
    std::vector<Vertex> vxs;
    std::vector<Eigen::Vector3d> vxs_position3d;

    if(obs.id_ == goal.obs ){ // if the goal obstacle is this one, then do not check, the algorithm already aims for the visible vxs
      continue;
    }

    // what if speed is 0 ????
    // Compute obstacle direction in x-y-t
    Eigen::Vector3d bb_direction;
    bb_direction[0] = obs.speed_*cos(obs.heading_);
    bb_direction[1] = obs.speed_*sin(obs.heading_);
    bb_direction[2] = 1;
    //bb_direction.normalize(); no, because multiplied by t* it returns the position at time t*

    obs.ComputeVertexes(v_info_.position, start.time, vxs);

    plotCKFile_ << "Obs_" << obs.id_ << std::endl;

    plotCKFile_ << "PlotIt" << std::endl;

    plotCKFile_ << "Direction_" << bb_direction(0) << "_" << bb_direction(1) << "_" << bb_direction(2) << std::endl;
    for(Vertex vx : vxs){
      plotCKFile_ << "Vx_" << vx.position[0] << "_" << vx.position[1] << std::endl;
      Eigen::Vector3d vx_pos(vx.position[0], vx.position[1], 0);
      vxs_position3d.push_back(vx_pos);
    }
    plotCKFile_ << "-" << std::endl;

    if(start.obs == "1" && start.vx == RR && goal.obs == "2" && goal.vx == FR){
      int stop = 1;
    }

    for( int i=0;i<4;i++){
      int idx1, idx2;
      switch (i) {
        case 0:
          idx1 = 0;
          idx2 = 2;
          break;
        case 1:
          idx1 = 0;
          idx2 = 1;
          break;
        case 2:
          idx1 = 3;
          idx2 = 2;
          break;
        case 3:
          idx1 = 3;
          idx2 = 1;
          break;
      }

      // if the path starts from the current obs, do not check the sides of the departing vx
      if(obs.id_ == start.obs && (start.vx == idx1 || start.vx == idx2)){
        continue;
      }

      Eigen::Vector3d bb_vector1, bb_vector2;
      bb_vector1 = vxs_position3d[idx1];
      bb_vector2 = vxs_position3d[idx2];

      // Get the plane normal
      Eigen::Vector3d side = bb_vector1 - bb_vector2;
      Eigen::Vector3d planeNormal = bb_direction.cross(side);
      planeNormal.normalize();

      // if the plane and the path are //
      double lookatme = planeNormal.dot(path);
      if (planeNormal.dot(path) == 0) continue; //do we need tolerance here?

      // Compute % of path where the collision happens (from http://paulbourke.net/geometry/pointlineplane/)
      Eigen::Vector3d startToOne = bb_vector1 - start3d;
      double u = planeNormal.dot(startToOne) / planeNormal.dot(path);

    	if (u>=0 && u<=1){ // intersect_point between start and goal
        // what about u==1 ? means the collision happens when reaching the vertex,
        //  that makes sense when approaching the obstacle I'm aiming to

        // Get collision time and point
        Eigen::Vector3d doable_path = path * u; // Part of the path prior the collision
        doable_path[2] = 0;
        double collision_time = doable_path.norm()/v_info_.speed; // When the collision happens
        doable_path[2] = collision_time;
        Eigen::Vector3d collision_point = start3d + doable_path; // Where the collision happens

        // Get vertexes at time t*
        Eigen::Vector3d vertex1_position = bb_vector1 + bb_direction * collision_time;
        Eigen::Vector3d vertex2_position = bb_vector2 + bb_direction * collision_time;

        // Check if point is inside those two vertexes
        Eigen::Vector3d P1 = collision_point - vertex1_position;
        Eigen::Vector3d P2 = collision_point - vertex2_position;
        if (P1.dot(P2) <= 0){
          return false;
        }
      }
    }
  }
  return true;
}

// Compute the path to reach the goal
bool path_planner::ComputePath(const Eigen::Vector2d& goal_position, std::stack<Node>& waypoints){

  // Log
  //std::stringstream ss;
  //ss << "log_goal_" << goal_position(1)  << "_" << goal_position(2) << ".txt";
  if(plotWpsFile_.is_open()){
    plotWpsFile_.close();
  }
  plotWpsFile_.open("WPlog.txt",std::ofstream::trunc);
  plotWpsFile_ << "Start_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
  plotWpsFile_ << "Goal_" << goal_position(0) << "_" << goal_position(1) << std::endl;

  bool found = false;
  // set of viable nodes, ordered by total cons (ascending)
  std::multiset<Node> nodes_set;
  // The planner starts from the vehicle position with time=0
  //std::shared_ptr<Node> root(new Node());
  Node root;
  root.position = v_info_.position;
  root.time = 0;
  UpdateCosts(root, 0, goal_position, 0);
  //root.parent = nullptr;
  nodes_set.insert(root);

  Node goal;
  goal.position = goal_position;
  goal.isGoal = true;
  Node current;
  //Node last_wp;
  // Run until every node is studied
  while(!nodes_set.empty()){
    current = *nodes_set.begin();
    nodes_set.erase(nodes_set.begin());

    if (CheckCollision(current, goal)){
      found = true;
      //last_wp = current;
      break;
    }

    for (Obstacle& obs : obss_info_.obstacles)
    {
      // Compute vertexes depending on distance
      std::vector<Vertex> vertexes;
      obs.ComputeVertexes(current.position, current.time, vertexes);
      // Compute when every visible vertex is intercepted
      ComputeInterceptPoints(current.position, obs, vertexes);
      // Check collision for every one of them
      for (const Vertex& vx : vertexes)
      {
        if(vx.isVisible && !(current.obs == obs.id_ && current.vx == vx.id)) {
          // Create new Node (still to check for collision)
          //shared_ptr<Node> newnode(new Node());
          Node newnode;
          newnode.position = vx.intercept_point.position;

          newnode.obs = obs.id_;
          newnode.vx = vx.id;
          newnode.time = current.time + vx.intercept_point.time;
          if (newnode.obs=="2" && newnode.vx==RR){
            int ahahah=1;
          }
          if (CheckCollision(current, newnode)) {
            // Save new node
            UpdateCosts(newnode, current.g, goal_position, vx.intercept_point.time);
            std::shared_ptr<Node> parent = std::make_shared<Node>(current);
            //std::shared_ptr<Node> parent = std::shared_ptr<Node>(current, [](const Node&) {});
            newnode.parent = parent;
            //current.children.push_back(newnode);
            nodes_set.insert(newnode);
          }
        }
      }
    }
  }

  if (found){
    // Build path from goal to initial position
    Node wp;
    Node* it = &current;

    // Log
    plotWpsFile_ << "Time_" << current.f << std::endl;
    plotWpsFile_ << "Waypoint_" << goal_position(0) << "_" << goal_position(1) << std::endl;
    for (Obstacle& obs : obss_info_.obstacles) {
      plotWpsFile_ << "Obs_" << obs.id_ << std::endl;
      Eigen::Vector2d pose = obs.ComputePosition(current.f);
      plotWpsFile_ << "Pose_" << pose(0) << "_" << pose(1) << std::endl;
      plotWpsFile_ << "Heading_" << obs.heading_ << std::endl;
      plotWpsFile_ << "Dimx_" << obs.dim_x_ << std::endl;
      plotWpsFile_ << "Dimy_" << obs.dim_y_ << std::endl;
      plotWpsFile_ << "Safety_" << obs.safety_bb_ratio_ << std::endl;
      plotWpsFile_ << "Max_" << obs.max_bb_ratio_ << std::endl;
      plotWpsFile_ << "-" << std::endl;
    }

    while(it->parent!=nullptr){
      //std::cout << "Obs: " << it->obs << std::endl;
      //std::cout << "Vx: " << it->vx << std::endl;
      //std::cout << "Time: " << it->time << std::endl;
      wp.position = it->position;
      wp.time = it->time;
      waypoints.push(wp);
      it = &*it->parent;

      // Log
      plotWpsFile_ << "Time_" << wp.time << std::endl;
      plotWpsFile_ << "Waypoint_" << wp.position(0) << "_" << wp.position(1) << std::endl;
      for (Obstacle& obs : obss_info_.obstacles) {
        plotWpsFile_ << "Obs_" << obs.id_ << std::endl;
        Eigen::Vector2d pose = obs.ComputePosition(wp.time);
        plotWpsFile_ << "Pose_" << pose(0) << "_" << pose(1) << std::endl;
        plotWpsFile_ << "Heading_" << obs.heading_ << std::endl;
        plotWpsFile_ << "Dimx_" << obs.dim_x_ << std::endl;
        plotWpsFile_ << "Dimy_" << obs.dim_y_ << std::endl;
        plotWpsFile_ << "Safety_" << obs.safety_bb_ratio_ << std::endl;
        plotWpsFile_ << "Max_" << obs.max_bb_ratio_ << std::endl;
        plotWpsFile_ << "-" << std::endl;
      }
    }

    // Log
    plotWpsFile_ << "Time_" << "0" << std::endl;
    plotWpsFile_ << "Waypoint_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
    for (Obstacle& obs : obss_info_.obstacles) {
      plotWpsFile_ << "Obs_" << obs.id_ << std::endl;
      Eigen::Vector2d pose = obs.ComputePosition(it->time);
      plotWpsFile_ << "Pose_" << pose(0) << "_" << pose(1) << std::endl;
      plotWpsFile_ << "Heading_" << obs.heading_ << std::endl;
      plotWpsFile_ << "Dimx_" << obs.dim_x_ << std::endl;
      plotWpsFile_ << "Dimy_" << obs.dim_y_ << std::endl;
      plotWpsFile_ << "Safety_" << obs.safety_bb_ratio_ << std::endl;
      plotWpsFile_ << "Max_" << obs.max_bb_ratio_ << std::endl;
      plotWpsFile_ << "-" << std::endl;
    }


    return true;
  }
  return false;
}


void path_planner::UpdateCosts(Node& node, double costToReachNode, const Eigen::Vector2d& goal, double timeShift){
  node.g = costToReachNode + timeShift;
  Eigen::Vector2d dist_to_goal = goal - node.position;
  node.h = dist_to_goal.norm()/v_info_.speed;
  node.f = node.g + node.h;
}