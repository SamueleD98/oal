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
    double heading_;
    double speed_;
    double dim_x_;
    double dim_y_;
    double max_bb_ratio_; // max_bb_dim / bb_dim
    double safety_bb_ratio_; // safety_bb_dim / bb_dim
    std::vector<Vertex> vxs; // local position (wrt obs)

    Eigen::Vector2d ComputePosition(double time){
      Eigen::Vector2d shift(speed_*time*cos(heading_), speed_*time*sin(heading_));
      return position_+shift;
    }

    void ComputeLocalVertexes(const Eigen::Vector2d& vhPos){
      // Distance obs-vehicle wrt obs frame, on x and y
      Eigen::Vector2d obs_vehicle = position_ - vhPos;
      Eigen::Rotation2D<double> rotation(heading_);
      Eigen::Vector2d bodyObs_obs_vehicle = rotation.inverse() * obs_vehicle;
      double dist_x = abs(bodyObs_obs_vehicle[0]);
      double dist_y = abs(bodyObs_obs_vehicle[1]);
      // Choose the bounding box dimension depending on the obs-vehicle distance
      double bb_dim_x, bb_dim_y;
      if (dist_x <= dim_x_ * safety_bb_ratio_ && dist_y <= dim_y_ * safety_bb_ratio_) {
        // Set safety b_box_dim if distance lesser than safety
        bb_dim_x = dim_x_ * safety_bb_ratio_;
        bb_dim_y = dim_y_ * safety_bb_ratio_;
      }else if (dist_x > dim_x_ * max_bb_ratio_ || dist_y > dim_y_ * max_bb_ratio_){
        // Set maximum b_box_dim if distance greater than maximum
        bb_dim_x = dim_x_ * max_bb_ratio_;
        bb_dim_y = dim_y_ * max_bb_ratio_;
      }else {
        // Set b_box_dim according to distance
        double ratio_x = dist_x/dim_x_;
        double ratio_y = dist_y/dim_y_;
        if (ratio_x >= ratio_y){
          bb_dim_x = dist_x;
          bb_dim_y = ratio_x * dim_y_;
        }else{
          bb_dim_x = ratio_y * dim_x_;
          bb_dim_y = dist_y;
        }
      }
      // Find the local vertexes position
      Vertex vx1, vx2, vx3, vx4;
      vx1.id = FR;
      vx1.position[0] = bb_dim_x/2;
      vx1.position[1] = -bb_dim_y/2;
      vx2.id = FL;
      vx2.position[0] = bb_dim_x/2;
      vx2.position[1] = bb_dim_y/2;
      vx3.id = RR;
      vx3.position[0] = -bb_dim_x/2;
      vx3.position[1] = -bb_dim_y/2;
      vx4.id = RL;
      vx4.position[0] = -bb_dim_x/2;
      vx4.position[1] = bb_dim_y/2;
      vxs.push_back(vx1);
      vxs.push_back(vx2);
      vxs.push_back(vx3);
      vxs.push_back(vx4);
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
