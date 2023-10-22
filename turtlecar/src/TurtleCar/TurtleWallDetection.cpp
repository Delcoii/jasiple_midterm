#include "TurtleCar/TurtleWallDetection.h"


double GetWallDist (double x, double y) {

    double d_top_dist = TURTLESIM_MAX_COORD - y;
    double d_bottom_dist = y;
    double d_left_dist = x;
    double d_right_dist = TURTLESIM_MAX_COORD - x;

    double min_dist = d_top_dist;
    if (min_dist > d_bottom_dist)   min_dist = d_bottom_dist;
    if (min_dist > d_left_dist)     min_dist = d_left_dist;
    if (min_dist > d_right_dist)    min_dist = d_right_dist;

    return min_dist;
}