#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug

// Vertex indexes map
enum vx_id {
    FR, // forward right
    FL, // forward left
    RR, // rear right
    RL  // rear left
};

struct Vertex {
    vx_id id;
    Eigen::Vector2d position;
    bool isVisible = false;
    Eigen::Vector2d ip_position;  //intercept point position
    double ip_time = -1; //intercept time
    //std::shared_ptr<Obstacle> obs = nullptr;
};

class Obstacle {
//private:
public:
    std::string id;
    Eigen::Vector2d position;  // the original position at time 0
    double heading;
    double speed;
    double dim_x;
    double dim_y;
    double bb_x_bow_ratio;
    double bb_x_stern_ratio;
    double bb_y_ratio;
    // double max_bb_ratio; // max_bb_dim / bb_dim
    double safety_bb_ratio; // safety_bb_dim / bb_dim
    std::vector<Vertex> vxs; // local position (wrt obs)

    static void SetSize(double dist_x, double dist_y, double dim_x, double dim_y, double safety_ratio, double bb_ratio_x, double bb_ratio_y, double &bb_dim_x, double &bb_dim_y){
      bool x_safety = (dist_x<=dim_x/2 * safety_ratio);
      bool y_safety = (dist_y<=dim_y/2 * safety_ratio);
      bool x_max = (dist_x>=dim_x/2 * bb_ratio_x);
      bool y_max = (dist_y>=dim_y/2 * bb_ratio_y);
      bool x_between = !x_safety && !x_max;
      bool y_between = !y_safety && !y_max;

      bb_dim_x = dim_x/2 * bb_ratio_x;
      bb_dim_y = dim_y * bb_ratio_y;

      if(x_safety && !y_max){
        bb_dim_x = dim_x/2 * safety_ratio;
      }
      if(y_safety && !x_max){
          bb_dim_y = dim_y * safety_ratio;
      }
      if(y_between && !x_max) {
        bb_dim_y = dist_y * 2;
      }
      if(x_between && !y_max){
        bb_dim_y = dist_x;
      }
    }

    void ComputeLocalVertexes(const Eigen::Vector2d &vhPos) {
      // Distance obs-vehicle wrt obs frame, on x and y
      Eigen::Vector2d vh_obs = vhPos - position;
      Eigen::Rotation2D<double> rotation(heading);
      Eigen::Vector2d bodyObs_vhPos = rotation.inverse() * vh_obs;
      double dist_x = abs(bodyObs_vhPos.x());
      double dist_y = abs(bodyObs_vhPos.y());
      double bb_dim_y, bb_dim_x_bow, bb_dim_x_stern;
      // asymmetric bb x dimension computation
      double theta = atan2(bodyObs_vhPos.y(), bodyObs_vhPos.x()); // error for (0,0)
      // choose depending on approaching angle
      bool IsAhead = (abs(theta)<=M_PI/2);
      if(IsAhead){
        bb_dim_x_stern = dim_x/2 * bb_x_stern_ratio;
        SetSize(dist_x, dist_y, dim_x, dim_y, safety_bb_ratio, bb_x_bow_ratio, bb_y_ratio, bb_dim_x_bow, bb_dim_y);
      }else{
        bb_dim_x_bow = dim_x/2 * bb_x_bow_ratio;
        SetSize(dist_x, dist_y, dim_x, dim_y, safety_bb_ratio, bb_x_stern_ratio, bb_y_ratio, bb_dim_x_stern, bb_dim_y);
      }
      // Find the local vertexes position
      Vertex vx1, vx2, vx3, vx4;
      vx1.id = FR;
      vx1.position[0] = bb_dim_x_bow;
      vx1.position[1] = -bb_dim_y / 2;
      vx2.id = FL;
      vx2.position[0] = bb_dim_x_bow;
      vx2.position[1] = bb_dim_y / 2;
      vx3.id = RR;
      vx3.position[0] = -bb_dim_x_stern;
      vx3.position[1] = -bb_dim_y / 2;
      vx4.id = RL;
      vx4.position[0] = -bb_dim_x_stern;
      vx4.position[1] = bb_dim_y / 2;
      vxs.push_back(vx1);
      vxs.push_back(vx2);
      vxs.push_back(vx3);
      vxs.push_back(vx4);
    }

//public:
    Obstacle(std::string name, Eigen::Vector2d position, double heading, double speed, double dim_x, double dim_y,
             double bb_y_ratio, double bb_x_bow_ratio, double bb_x_stern_ratio, double safety_bb_ratio)
            : id(std::move(name)), position(std::move(position)), heading(heading), speed(speed), dim_x(dim_x),
              dim_y(dim_y), bb_y_ratio(bb_y_ratio), bb_x_bow_ratio(bb_x_bow_ratio), bb_x_stern_ratio(bb_x_stern_ratio), safety_bb_ratio(safety_bb_ratio) {

      if (dim_x <= 0 || dim_y <= 0) {
        throw std::invalid_argument("Obstacle dimension must be strictly positive.");
      }

      if (safety_bb_ratio<1 || bb_y_ratio<1 || bb_x_bow_ratio<1 || bb_x_stern_ratio<1 ) {
        throw std::invalid_argument("Bounding box ratios cannot be less than one.");
      }

      if (safety_bb_ratio>bb_y_ratio || safety_bb_ratio>bb_x_bow_ratio || safety_bb_ratio>bb_x_stern_ratio) {
        throw std::invalid_argument("Generic bounding box ratios cannot be smaller than safety one.");
      }

      if (heading < 0 || heading > 2*M_PI) {
        throw std::invalid_argument("Obstacle heading must be expressed in radians [0, 2*pi]");
      }
    }
};

struct Node {
    Eigen::Vector2d position; //vehicle position
    // time and costToReach are the same when the cost==time to reach the target (should we be able to manage other costs to minimize?)
    double time = -1;  // time instant
    double costToReach = -1; //cost to reach the Node
    double costToGoal = -1; //estimated cost to reach Goal
    double costTotal = -1; //g+h total cost
    std::string obs;
    double obs_heading = -1;
    vx_id vx;
    std::shared_ptr<Node> parent = nullptr;
    std::vector<vx_id> colregsLimitedVxs;

    // Used to order nodes in set according to total cost to reach the goal
    bool operator<(const Node &other) const {
      return costTotal < other.costTotal;
    }
};

struct VehicleInfo {
    Eigen::Vector2d position;
    double speed;
};

struct ObstaclesInfo {
    std::vector<Obstacle> obstacles;
};


#endif
