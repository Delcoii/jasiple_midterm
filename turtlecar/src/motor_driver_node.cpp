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

#include "turtlecar/remote_sig_mapping.h"

#define LOOP_HZ             60.


// max velocity for turtle
#define GEAR1_MAX_VEL       1.0
#define GEAR2_MAX_VEL       3.0
#define GEAR3_MAX_VEL       5.0


int g_steer_us = (int)STEER_SIG_N;
int g_accel_us = (int)ACCEL_SIG_N;
void SteeringCallback(const std_msgs::Int32::ConstPtr& msg);
void AccelCallback(const std_msgs::Int32::ConstPtr& msg);

double map(double x, double in_min, double in_max, double out_min, double out_max);
double constrain(double x, double out_min, double out_max);

int main(int argc, char** argv) {

    ros::init(argc, argv, "motor_driver");

    ros::NodeHandle nh;
    ros::Time::init();
    ros::Rate loop_rate_hz(LOOP_HZ);

    ros::Subscriber steer_signal = nh.subscribe("/remocon_steer_us", 100, SteeringCallback);
    ros::Subscriber accel_signal = nh.subscribe("/remocon_accel_us", 100, AccelCallback);
    ros::Publisher turtle_cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    
    geometry_msgs::Twist turtle_cmd;

    while (ros::ok()) {
        // initialize global var to local
        double d_steer_sig_us = (double)g_steer_us;
        double d_accel_sig_us = (double)g_accel_us;

        RemoteSigMapping(d_steer_sig_us, d_accel_sig_us);
        


        // set speed(Twist) for turtlesim (gear 2)
        // >>>!!  angular direction (mapping reverse) !!<<<  
        double d_angular_speed = map(d_steer_sig_us, STEER_SIG_MIN, STEER_SIG_MAX, 
                                    GEAR2_MAX_VEL, (-1.)*GEAR2_MAX_VEL);

        double d_linear_speed = map(d_accel_sig_us, ACCEL_SIG_MIN, ACCEL_SIG_MAX, 
                                    (-1.)*GEAR2_MAX_VEL, GEAR2_MAX_VEL);


        turtle_cmd.linear.x = d_linear_speed;
        turtle_cmd.linear.y = 0.;
        turtle_cmd.linear.z = 0.;
        turtle_cmd.angular.x = 0.;
        turtle_cmd.angular.y = 0.;
        turtle_cmd.angular.z = d_angular_speed;
        
        turtle_cmd_pub.publish(turtle_cmd);
        // for debugging
        std::cout <<
            "================================" << "\n" <<
            "motor driver output" << "\n\n" << 
            "    steer sig : " << d_steer_sig_us << "\n" <<
            "    accel sig : " << d_accel_sig_us << "\n" << 
            "    linear x   : " << d_linear_speed << "\n" <<
            "    angular z  : " << d_angular_speed << "\n" <<
        std::endl;

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

