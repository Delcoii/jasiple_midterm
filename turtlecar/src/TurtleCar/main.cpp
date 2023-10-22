/*
 * Author   : 박성훈
 * Date     : 23.10.17
 * 
 * Input    : /remocon_steer_us
 *            /remocon_accel_us
 * 
 * Output   : (geometry_twist)
 * 
 * RC카 리모콘 신호를 subscribe하여 turtlesim turtle을 조작함
 * 이외에 speed controller(service), wall sensor(action) 등으로 속도가 자동으로 조절됨
 * 
*/

#include <iostream>
#include <ros/ros.h>

#include "TurtleCar/TurtleCar.h"
#include "SafetyPlanner/SafetyPlan.h"               // action server class

#define LOOP_HZ             60.


int main(int argc, char** argv) {

    ros::init(argc, argv, "turtlecar");
    ros::Time::init();
    ros::Rate loop_rate_hz(LOOP_HZ);

    /*  TurtleCar class info
        sub             : steer, accel sig
        pub             : turtle pose, moving command
        service server  : gear shift
    */
    TurtleCar turtlecar;
    SafetyPlanAction safety_plan("/wall_detect");


    while (ros::ok()) {
        
        if (safety_plan.Danger() == true) {
            turtlecar.Run(true);    // slow mode enable
        }
        else {
            turtlecar.Run();
        }
        
        safety_plan.SetWallDist(turtlecar.WallDist()); 

        loop_rate_hz.sleep();
        ros::spinOnce();
    }
}

