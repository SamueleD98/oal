#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <stack>
#include "oal/data_structs/obstacle.hpp"
#include "oal/data_structs/node.hpp"

typedef std::shared_ptr<Obstacle> obs_ptr;

struct VehicleInfo {
    Eigen::Vector2d position;
    std::vector<double> velocities;
};

struct ObstaclesInfo {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
};

struct Path {
    std::stack<Node> waypoints;
    std::vector<std::string> overtakingObsList;
    std::vector<std::string> overtakenObsList;

    bool empty() const{
      return waypoints.empty();
    }

    Node top(){
      return waypoints.top();
    }

    void pop(){
      // TODO still necessary?
      // When reached a node, delete it from Path and set the obstacle as overtaken to avoid future crossing
      Node nd = waypoints.top();
      if(nd.obs_ptr != nullptr){
        auto pos = std::find(overtakingObsList.begin(), overtakingObsList.end(), nd.obs_ptr->id);
        if (pos != overtakingObsList.end())  overtakenObsList.push_back(nd.obs_ptr->id);
      }
      waypoints.pop();
    }
};


#endif
