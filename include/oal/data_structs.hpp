#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>

// Vertex indexes map
enum vx_id {
  FR, // forward right
  FL, // forward left
  RR, // rear right
  RL  // rear left
};

struct Node {
  Eigen::Vector2d position; //vehicle position
  double time = -1;  // time instant
  double g = -1; //cost to reach the Node
  double h = -1; //estimated cost to reach Goal
  double f = -1; //g+h total cost
  std::string obs = "";
  vx_id vx;
  bool isGoal = false;
  std::shared_ptr<Node> parent = nullptr;
  // Order nodes in set according to total cost to reach the goal
  bool operator<(const Node other) const {
    return f < other.f;
  }
};

struct InterceptPoint{
  Eigen::Vector2d position;
  double time{};
};
struct Vertex {
  vx_id id;
  Eigen::Vector2d position;
  bool isVisible;
  InterceptPoint intercept_point;
};

struct Obstacle {
//private:
  std::string id_;
  Eigen::Vector2d position_;
  double heading_; // does not change between local and latlong, right?
  double speed_;
  double dim_x_;
  double dim_y_;
  double max_bb_ratio_ = 2; // max_bb_dim / bb_dim
  double safety_bb_ratio_ = 1.6; // safety_bb_dim / bb_dim

  // How the position changes in time from the original one
  Eigen::Vector2d ComputePosition(double time){
      Eigen::Vector2d shift(speed_*time*cos(heading_), speed_*time*sin(heading_));
      return position_+shift;
  }

  // Compute the vertexes according to the distance from the vehicle and a time instant
  void ComputeVertexes(const Eigen::Vector2d& vhPos, double time, std::vector<Vertex>& vertexes){
    // Get the position at specific time instant
    Eigen::Vector2d obs_position = ComputePosition(time);
    // Distance obs-vehicle wrt obs frame, on x and y
    Eigen::Vector2d obs_vehicle = obs_position - vhPos;
    Eigen::Rotation2D<double> rotation(heading_);
    Eigen::Vector2d bodyObs_obs_vehicle = rotation.inverse() * obs_vehicle;
    double dist_x = abs(bodyObs_obs_vehicle[0]);
    double dist_y = abs(bodyObs_obs_vehicle[1]);
    // Choose the bounding box dimension depending on the obs-vehicle distance
    double current_dim_x, current_dim_y;
    if (dist_x <= dim_x_ * safety_bb_ratio_ && dist_y <= dim_y_ * safety_bb_ratio_) {
      // Set safety b_box_dim if distance lesser than safety
      current_dim_x = dim_x_ * safety_bb_ratio_;
      current_dim_y = dim_y_ * safety_bb_ratio_;
    }else if (dist_x > dim_x_ * max_bb_ratio_ || dist_y > dim_y_ * max_bb_ratio_){
      // Set maximum b_box_dim if distance greater than maximum
      current_dim_x = dim_x_ * max_bb_ratio_;
      current_dim_y = dim_y_ * max_bb_ratio_;
    }else {
       // Set b_box_dim according to distance
       double ratio_x = dist_x/dim_x_;
       double ratio_y = dist_y/dim_y_;
       if (ratio_x >= ratio_y){
         current_dim_x = dist_x;
         current_dim_y = ratio_x * dim_y_;
       }else{
         current_dim_y = dist_y;
         current_dim_x = ratio_y * dim_x_;
       }
    }
    // Find the vertexes position and visibility
    std::vector<Eigen::Vector2d> vertexes_vehicle;  // distances in absolute frame
    std::vector<double> theta;  // angles wrt abs frame
    double sin_heading = std::sin(heading_);
    double cos_heading = std::cos(heading_);
    // Find the vertexes position
    std::map<int, std::pair<vx_id, std::vector<int>>> vx_id_map;
    vx_id_map[0] = std::make_pair(FR, std::vector<int>({1, 1}));
    vx_id_map[1] = std::make_pair(FL, std::vector<int>({1, -1}));
    vx_id_map[2] = std::make_pair(RR, std::vector<int>({-1, 1}));
    vx_id_map[3] = std::make_pair(RL, std::vector<int>({-1, -1}));

    for(int i=0; i<4; i++){
      Vertex vx;
      vx.id = vx_id_map[i].first;
      vx.isVisible = false;
      vx.position[0] = obs_position[0]
                    + vx_id_map[i].second[0] * current_dim_x / 2 * cos_heading
                    + vx_id_map[i].second[1] * current_dim_y / 2 * sin_heading;
      vx.position[1] = obs_position[1]
                    + vx_id_map[i].second[0] * current_dim_x / 2 * sin_heading
                    - vx_id_map[i].second[1] * current_dim_y / 2 * cos_heading;
      vertexes.push_back(vx);
      vertexes_vehicle.push_back(vx.position - vhPos); //if 0 there's a problem
      theta.push_back(std::atan2(vertexes_vehicle[i][1], vertexes_vehicle[i][0]));
    }
    // Find which are visible
    auto max_v_ptr = std::max_element(theta.begin(), theta.end());
    int max_v = std::distance(theta.begin(), max_v_ptr);
    auto min_v_ptr = std::min_element(theta.begin(), theta.end());
    int min_v = std::distance(theta.begin(), min_v_ptr);
    // min/max angle vertexes are visible
    vertexes[min_v].isVisible = true;
    vertexes[max_v].isVisible = true;
    // are also visible vertexes which distance is smaller then those two
    double distance, min_v_distance, max_v_distance;
    min_v_distance = vertexes_vehicle[min_v].norm();
    max_v_distance = vertexes_vehicle[max_v].norm();
    for (int i=0; i<4; i++){
      if( i==min_v || i==max_v){
        continue;
      }
      distance = vertexes_vehicle[i].norm();
      if( distance < min_v_distance && distance < max_v_distance){
        vertexes[i].isVisible = true;
      }
    }
  }
};

struct VehicleInfo{
  Eigen::Vector2d position;
  double speed;
};

struct ObstaclesInfo{
  std::vector<Obstacle> obstacles;
};


#endif
