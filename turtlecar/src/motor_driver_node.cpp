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
 * 이외에 speed controller(기어박스), wall sensor 등으로 속도가 자동으로 조절됨
 * 
*/

#include <iostream>
#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim_msgs/Pose.h"                    // topics

#include "turtlecar/SetGearShift.h"                 // service 

#include "turtlecar/SafetyPlanner/SafetyPlan.h"     // action server class

#include "turtlecar/remote_sig_mapping.h"
#include "turtlecar/turtle_wall_detection.h"        // calculating wall distance


#define LOOP_HZ             60.


// max velocity for turtle
#define DANGER_MAX_VEL      0.5
#define GEAR1_MAX_VEL       1.0
#define GEAR2_MAX_VEL       4.0
#define GEAR3_MAX_VEL       7.0


int g_steer_us = (int)STEER_SIG_N;
int g_accel_us = (int)ACCEL_SIG_N;
void SteeringCallback(const std_msgs::Int32::ConstPtr& msg);
void AccelCallback(const std_msgs::Int32::ConstPtr& msg);

double g_turtle_pose_x = 5.544445;
double g_turtle_pose_y = 5.544445;
void TurtlePoseCallback(const turtlesim_msgs::Pose::ConstPtr& msg);

int g_gear = 2;
bool GearShiftCallback(turtlecar::SetGearShift::Request &req, turtlecar::SetGearShift::Response &res);

double map(double x, double in_min, double in_max, double out_min, double out_max);
double constrain(double x, double out_min, double out_max);



int main(int argc, char** argv) {

    ros::init(argc, argv, "motor_driver");

    ros::NodeHandle nh;
    ros::Time::init();
    ros::Rate loop_rate_hz(LOOP_HZ);

    ros::Subscriber steer_signal = nh.subscribe("/remocon_steer_us", 100, SteeringCallback);
    ros::Subscriber accel_signal = nh.subscribe("/remocon_accel_us", 100, AccelCallback);
    ros::Subscriber turtle_pose = nh.subscribe("/turtle1/pose", 100, TurtlePoseCallback);

    ros::Publisher turtle_cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    geometry_msgs::Twist turtle_cmd;

    ros::ServiceServer service = nh.advertiseService("/gear_shift", GearShiftCallback);

    SafetyPlanAction safety_plan("/wall_detect");

    double d_angular_speed = -1.;
    double d_linear_speed = -1.;
    double d_steer_sig_us = -1.;
    double d_accel_sig_us = -1.;
    int i_turtlecar_gear = -1.;
    double d_turtle_pose_x = -1.;
    double d_turtle_pose_y = -1.;
    double d_wall_dist = -1.;

    while (ros::ok()) {
        // initialize global var to local
        d_steer_sig_us = (double)g_steer_us;
        d_accel_sig_us = (double)g_accel_us;
        i_turtlecar_gear = g_gear;
        d_turtle_pose_x = g_turtle_pose_x;
        d_turtle_pose_y = g_turtle_pose_y;


        RemoteSigMapping(d_steer_sig_us, d_accel_sig_us);
        
        if (safety_plan.Danger() == true) {
            d_angular_speed = map(d_steer_sig_us, STEER_SIG_MIN, STEER_SIG_MAX, DANGER_MAX_VEL, (-1.)*DANGER_MAX_VEL);

            d_linear_speed = map(d_accel_sig_us, ACCEL_SIG_MIN, ACCEL_SIG_MAX, (-1.)*DANGER_MAX_VEL, DANGER_MAX_VEL);
        }
        else if (i_turtlecar_gear == 1) {
            d_angular_speed = map(d_steer_sig_us, STEER_SIG_MIN, STEER_SIG_MAX, GEAR1_MAX_VEL, (-1.)*GEAR1_MAX_VEL);

            d_linear_speed = map(d_accel_sig_us, ACCEL_SIG_MIN, ACCEL_SIG_MAX, (-1.)*GEAR1_MAX_VEL, GEAR1_MAX_VEL);
        }
        else if (i_turtlecar_gear == 2) {
            d_angular_speed = map(d_steer_sig_us, STEER_SIG_MIN, STEER_SIG_MAX, GEAR2_MAX_VEL, (-1.)*GEAR2_MAX_VEL);

            d_linear_speed = map(d_accel_sig_us, ACCEL_SIG_MIN, ACCEL_SIG_MAX, (-1.)*GEAR2_MAX_VEL, GEAR2_MAX_VEL);
        }
        else if (i_turtlecar_gear == 3) {
            d_angular_speed = map(d_steer_sig_us, STEER_SIG_MIN, STEER_SIG_MAX, GEAR3_MAX_VEL, (-1.)*GEAR3_MAX_VEL);

            d_linear_speed = map(d_accel_sig_us, ACCEL_SIG_MIN, ACCEL_SIG_MAX, (-1.)*GEAR3_MAX_VEL, GEAR3_MAX_VEL);
        }


        turtle_cmd.linear.x = d_linear_speed;
        turtle_cmd.linear.y = 0.;
        turtle_cmd.linear.z = 0.;
        turtle_cmd.angular.x = 0.;
        turtle_cmd.angular.y = 0.;
        turtle_cmd.angular.z = d_angular_speed;

        d_wall_dist = WallDist(d_turtle_pose_x, d_turtle_pose_y);
        safety_plan.SetWallDist(d_wall_dist);
        
        turtle_cmd_pub.publish(turtle_cmd);
        // for debugging
        // std::cout <<
        // "================================" << "\n" <<
        //     "motor driver output" << "\n\n" << 
        //     "    steer sig : " << d_steer_sig_us << "\n" <<
        //     "    accel sig : " << d_accel_sig_us << "\n" << 
        //     "    linear x   : " << d_linear_speed << "\n" <<
        //     "    angular z  : " << d_angular_speed << "\n" <<
        // "    gear       : " << i_turtlecar_gear << "\n" <<
        // "    wall dist  : " << d_wall_dist << "\n" <<
        // std::endl;

        loop_rate_hz.sleep();
        ros::spinOnce();
    }

}

void SteeringCallback(const std_msgs::Int32::ConstPtr& msg) {
    g_steer_us = msg->data;
}
void AccelCallback(const std_msgs::Int32::ConstPtr& msg) {
    g_accel_us = msg->data;
}
void TurtlePoseCallback(const turtlesim_msgs::Pose::ConstPtr& msg) {
    g_turtle_pose_x = msg->x;
    g_turtle_pose_y = msg->y;
}

bool GearShiftCallback(turtlecar::SetGearShift::Request &req, turtlecar::SetGearShift::Response &res) {
    res.gear_before = g_gear;
    g_gear = req.gear;
    res.gear_after = g_gear;

    ROS_INFO("\ngear input : %d\nsending gear status %d -> %d", (int)req.gear, (int)res.gear_before, (int)res.gear_after);

    return true;
}