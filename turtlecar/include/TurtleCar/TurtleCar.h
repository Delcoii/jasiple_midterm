/*
 * Author   : 박성훈
 * Date     : 23.10.17
 * 
 * Input    : /remocon_steer_us
 *            /remocon_accel_us
 *            /turtle1/pose
 * 
 * Output   : (geometry_twist)
 * 
 * RC카 리모콘 신호를 subscribe하여 turtlesim turtle을 조작함
 * speed controller(service server) 실행
 * wall distance 계산
 * 
*/


#ifndef __TURTLE_CAR_H__
#define __TURTLE_CAR_H__

#include <iostream>
#include <ros/ros.h>

#include "TurtleCar/RemoteSigMapping.h"
#include "TurtleCar/TurtleWallDetection.h"      // calculating wall distance

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim_msgs/Pose.h"

#include "turtlecar/SetGearShift.h"             // service file


// max velocity for turtle
#define DANGER_MAX_VEL      0.5
#define GEAR1_MAX_VEL       1.0
#define GEAR2_MAX_VEL       4.0
#define GEAR3_MAX_VEL       7.0

class TurtleCar {
protected:
    ros::NodeHandle nh;

    ros::Subscriber steer_signal = nh.subscribe("/remocon_steer_us", 100, &TurtleCar::SteeringCallback, this);
    void SteeringCallback(const std_msgs::Int32::ConstPtr& msg);

    ros::Subscriber accel_signal = nh.subscribe("/remocon_accel_us", 100, &TurtleCar::AccelCallback, this);
    void AccelCallback(const std_msgs::Int32::ConstPtr& msg);
    
    ros::Subscriber turtle_pose = nh.subscribe("/turtle1/pose", 100, &TurtleCar::TurtlePoseCallback, this);
    void TurtlePoseCallback(const turtlesim_msgs::Pose::ConstPtr& msg);

    ros::ServiceServer service = nh.advertiseService("/gear_shift", &TurtleCar::GearShiftCallback, this);
    bool GearShiftCallback(turtlecar::SetGearShift::Request &req, turtlecar::SetGearShift::Response &res);

    ros::Publisher turtle_cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);




    int m_steer_us;
    int m_accel_us;
    double m_turtle_pose_x;
    double m_turtle_pose_y;
    int m_gear;

    geometry_msgs::Twist m_turtle_cmd;
    double m_wall_dist;

public:
    TurtleCar();
    void Run(bool slow = false);
    void PrintInfo();
    double WallDist();

};




#endif // __TURTLE_CAR_H__