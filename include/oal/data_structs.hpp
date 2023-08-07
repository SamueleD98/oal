#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>

// Vertex indexes map
enum vx_id {
  FR, // forward right
  FL, // forward left
  RR, // rear right
  RL  // rear left
};

struct Node {
  Eigen::Vector2d position; //vehicle position
  // time and costToReach are the same when the cost==time to reach the target (should we be able to manage other costs to minimize?)
  double time = -1;  // time instant
  double costToReach = -1; //cost to reach the Node
  double costToGoal = -1; //estimated cost to reach Goal
  double costTotal = -1; //g+h total cost
  std::string obs = "";
  vx_id vx;
  std::shared_ptr<Node> parent = nullptr;
  // Used to order nodes in set according to total cost to reach the goal
  bool operator<(const Node other) const {
    return costTotal < other.costTotal;
  }
};

struct Vertex {
  vx_id id;
  Eigen::Vector2d position;
  bool isVisible = false;
  Eigen::Vector2d ip_position;  //intercept point position
  double ip_time = -1; //intercept time
};

class Obstacle {
//private:
public:
  std::string id_;
  Eigen::Vector2d position_;  // the original position at time 0
  double heading_; // does not change between local and latlong, right?
  double speed_;
  double dim_x_;
  double dim_y_;
  double max_bb_ratio_; // max_bb_dim / bb_dim
  double safety_bb_ratio_; // safety_bb_dim / bb_dim
  double bb_dim_x_;
  double bb_dim_y_;

  void ComputeBBDimension(const Eigen::Vector2d& vhPos){
    // Distance obs-vehicle wrt obs frame, on x and y
    Eigen::Vector2d obs_vehicle = position_ - vhPos;
    Eigen::Rotation2D<double> rotation(heading_);
    Eigen::Vector2d bodyObs_obs_vehicle = rotation.inverse() * obs_vehicle;
    double dist_x = abs(bodyObs_obs_vehicle[0]);
    double dist_y = abs(bodyObs_obs_vehicle[1]);
    // Choose the bounding box dimension depending on the obs-vehicle distance
    double current_dim_x, current_dim_y;
    if (dist_x <= dim_x_ * safety_bb_ratio_ && dist_y <= dim_y_ * safety_bb_ratio_) {
      // Set safety b_box_dim if distance lesser than safety
      bb_dim_x_ = dim_x_ * safety_bb_ratio_;
      bb_dim_y_ = dim_y_ * safety_bb_ratio_;
    }else if (dist_x > dim_x_ * max_bb_ratio_ || dist_y > dim_y_ * max_bb_ratio_){
      // Set maximum b_box_dim if distance greater than maximum
      bb_dim_x_ = dim_x_ * max_bb_ratio_;
      bb_dim_y_ = dim_y_ * max_bb_ratio_;
    }else {
      // Set b_box_dim according to distance
      double ratio_x = dist_x/dim_x_;
      double ratio_y = dist_y/dim_y_;
      if (ratio_x >= ratio_y){
        bb_dim_x_ = dist_x;
        bb_dim_y_ = ratio_x * dim_y_;
      }else{
        bb_dim_x_ = ratio_y * dim_x_;
        bb_dim_y_ = dist_y;
      }
    }
  }

  // Compute the vertexes according to the distance from the vehicle and a time instant
  void ComputeVertexes(const Eigen::Vector2d& vhPos, double time, std::vector<Vertex>& vertexes){
    // Get the position at specific time instant
    Eigen::Vector2d shift(speed_*time*cos(heading_), speed_*time*sin(heading_));
    Eigen::Vector2d obs_position = position_+shift;

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
      vx.position[0] = obs_position[0]
                    + vx_id_map[i].second[0] * bb_dim_x_ / 2 * cos_heading
                    + vx_id_map[i].second[1] * bb_dim_y_ / 2 * sin_heading;
      vx.position[1] = obs_position[1]
                    + vx_id_map[i].second[0] * bb_dim_x_ / 2 * sin_heading
                    - vx_id_map[i].second[1] * bb_dim_y_ / 2 * cos_heading;
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

//public:
    Obstacle(std::string id, Eigen::Vector2d position, double heading, double speed, double dim_x, double dim_y, double max_bb_ratio, double safety_bb_ratio )
    : id_(std::move(id)), position_(std::move(position)), heading_(heading), speed_(speed), dim_x_(dim_x), dim_y_(dim_y), max_bb_ratio_(max_bb_ratio), safety_bb_ratio_(safety_bb_ratio)
    {
      if (dim_x <= 0 || dim_y <= 0) {
        throw std::invalid_argument("Obstacle dimension cannot be negative.");
      }

      if (safety_bb_ratio > max_bb_ratio) {
        throw std::invalid_argument("Safety bounding box ratio cannot be greater than the maximum bounding box one");
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
