#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>

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
    double max_bb_ratio; // max_bb_dim / bb_dim
    double safety_bb_ratio; // safety_bb_dim / bb_dim
    std::vector<Vertex> vxs; // local position (wrt obs)

    void ComputeLocalVertexes(const Eigen::Vector2d &vhPos) {
      // Distance obs-vehicle wrt obs frame, on x and y
      Eigen::Vector2d obs_vehicle = position - vhPos;
      Eigen::Rotation2D<double> rotation(heading);
      Eigen::Vector2d bodyObs_obs_vehicle = rotation.inverse() * obs_vehicle;
      double dist_x = abs(bodyObs_obs_vehicle[0]);
      double dist_y = abs(bodyObs_obs_vehicle[1]);
      // Choose the bounding box dimension depending on the obs-vehicle distance
      double bb_dim_x, bb_dim_y;
      if (dist_x <= dim_x * safety_bb_ratio && dist_y <= dim_y * safety_bb_ratio) {
        // Set safety b_box_dim if distance lesser than safety
        bb_dim_x = dim_x * safety_bb_ratio;
        bb_dim_y = dim_y * safety_bb_ratio;
      } else {
        if (dist_x > dim_x * max_bb_ratio || dist_y > dim_y * max_bb_ratio) {
          // Set maximum b_box_dim if distance greater than maximum
          bb_dim_x = dim_x * max_bb_ratio;
          bb_dim_y = dim_y * max_bb_ratio;
        } else {
          // Set b_box_dim according to distance
          double ratio_x = dist_x / dim_x;
          double ratio_y = dist_y / dim_y;
          if (ratio_x >= ratio_y) {
            bb_dim_x = dist_x;
            bb_dim_y = ratio_x * dim_y;
          } else {
            bb_dim_x = ratio_y * dim_x;
            bb_dim_y = dist_y;
          }
        }
      }
      // Find the local vertexes position
      Vertex vx1, vx2, vx3, vx4;
      vx1.id = FR;
      vx1.position[0] = bb_dim_x / 2;
      vx1.position[1] = -bb_dim_y / 2;
      vx2.id = FL;
      vx2.position[0] = bb_dim_x / 2;
      vx2.position[1] = bb_dim_y / 2;
      vx3.id = RR;
      vx3.position[0] = -bb_dim_x / 2;
      vx3.position[1] = -bb_dim_y / 2;
      vx4.id = RL;
      vx4.position[0] = -bb_dim_x / 2;
      vx4.position[1] = bb_dim_y / 2;
      vxs.push_back(vx1);
      vxs.push_back(vx2);
      vxs.push_back(vx3);
      vxs.push_back(vx4);
    }

//public:
    Obstacle(std::string name, Eigen::Vector2d position, double heading, double speed, double dim_x, double dim_y,
             double max_bb_ratio, double safety_bb_ratio)
            : id(std::move(name)), position(std::move(position)), heading(heading), speed(speed), dim_x(dim_x),
              dim_y(dim_y), max_bb_ratio(max_bb_ratio), safety_bb_ratio(safety_bb_ratio) {

      if (dim_x <= 0 || dim_y <= 0) {
        throw std::invalid_argument("Obstacle dimension must be strictly positive.");
      }

      if (safety_bb_ratio < 1 || max_bb_ratio < 1) {
        throw std::invalid_argument("Safety and maximum bounding box ratios cannot be less than one.");
      }

      if (safety_bb_ratio > max_bb_ratio) {
        throw std::invalid_argument("Safety bounding box ratio cannot be greater than the maximum bounding box one.");
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
    std::string obs = "";
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
