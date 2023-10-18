/*
 * motor driver의 기어를 설정함(변속기)
 * 1단, 2단, 3단 존재
 * service의 client 역할 노드 (motor_driver node가 server)
 * 
 * input    : desired gear stage
 * output   : gear stage before & after
 * 
 */

#include <iostream>
#include <ros/ros.h>

#include "turtlecar/SetGearShift.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "transmission");

    if (argc != 2) {
        ROS_INFO("usage :  transmission (1~3)");
        return 1;
    }

    char gear_input = atoll(argv[1]);
    if ((gear_input < 1) && (gear_input > 3)) {
        ROS_INFO("usage : transmission (1~3)");
        return 1;
    }

    ros::NodeHandle nh;

    // Send 
    // Get gear stage before & after
    ros::ServiceClient client = nh.serviceClient<turtlecar::SetGearShift>("/gear_shift");
    turtlecar::SetGearShift srv;

    // gear input
    srv.request.gear = gear_input;

    if (client.call(srv)) {
        ROS_INFO("Gear changed by %d to %d", (int)srv.response.gear_before,
                                            (int)srv.response.gear_after);
    }
    else {
        ROS_ERROR("Failed to call service /gear_shift");
        return 1;
    }

    return 0;
   
}