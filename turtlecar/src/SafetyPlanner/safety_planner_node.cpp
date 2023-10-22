/*
 * turtle이 벽과 가까워지면 motor driver를 저속으로 변경함
 * 설정 거리 진입 시 작동함
 * 
 * input    : 설정 거리
 * output   : 
 * 
 */

#include <iostream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "turtlecar/WallDetectAction.h"



#define DANGER_DEFAULT      0.5
#define TIMEOUT_SEC         3600.

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "safety_planner");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<turtlecar::WallDetectAction> ac("/wall_detect", true);

    ROS_INFO("Waiting for action server(motor driver node) to start");
    ac.waitForServer();

    ROS_INFO("Sending danger reference distance to motor driver");
    
    // read by parameter
    double danger_dist;
    nh.param("/danger_zone_dist", danger_dist, DANGER_DEFAULT);

    turtlecar::WallDetectGoal goal;
    goal.danger_distance = danger_dist;

    ac.sendGoal(goal);
    std::cout << "timeout(s) : " << TIMEOUT_SEC << std::endl;

    bool finished = ac.waitForResult(ros::Duration(TIMEOUT_SEC));

    if (finished == true) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Turtlecar is now slow!!(%s)", state.toString().c_str());
    }
    else {
        ROS_INFO("Action did not finish before the timeout");
    }

    return 0;
}