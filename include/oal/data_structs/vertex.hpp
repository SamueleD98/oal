#ifndef OAL_VERTEX_HPP
#define OAL_VERTEX_HPP


#include "oal/data_structs/tpoint.hpp"

// Vertex indexes map
enum vx_id {
    FR = 0, // forward right
    FL = 1, // forward left
    RR = 2, // rear right
    RL = 3, // rear left
    NA = 5  // NOT ASSIGNED, Default
};

struct Vertex {
    vx_id id;
    Eigen::Vector2d position; //absolute
    bool isVisible = false; //visibility from own ship
    TPoint intercept_point;
    /*Eigen::Vector2d ip_position;  //intercept point position (absolute)
    double ip_time = -1; //intercept time*/
};


#endif //OAL_VERTEX_HPP
