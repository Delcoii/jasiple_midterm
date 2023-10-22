#include "TurtleCar/TurtleCar.h"


TurtleCar::TurtleCar() {
    m_steer_us = (int)STEER_SIG_N;
    m_accel_us = (int)ACCEL_SIG_N;
    m_turtle_pose_x = 5.544445;
    m_turtle_pose_y = 5.544445;
    m_gear = 2;

    m_turtle_cmd.linear.x = -1.;
    m_turtle_cmd.linear.y = -1.;
    m_turtle_cmd.linear.z = -1.;
    m_turtle_cmd.angular.x = -1.;
    m_turtle_cmd.angular.y = -1.;
    m_turtle_cmd.angular.z = -1.;
    m_wall_dist = -1.;
}

// 아두이노 + 리모콘에 나오는 신호를 기어/안전모드 에 따라 매핑
void TurtleCar::Run(bool slow) {
    // initialize member var to local
    double d_angular_speed = -1;
    double d_linear_speed = -1.;

    double d_steer_sig_us = m_steer_us;
    double d_accel_sig_us = m_accel_us;
    int i_turtlecar_gear = m_gear;
    double d_turtle_pose_x = m_turtle_pose_x;
    double d_turtle_pose_y = m_turtle_pose_y;


    RemoteSigMapping(d_steer_sig_us, d_accel_sig_us);

    if (slow == true) {
        d_angular_speed = map(d_steer_sig_us, STEER_SIG_MIN, STEER_SIG_MAX, DANGER_MAX_VEL, (-1.)*DANGER_MAX_VEL);

        d_linear_speed = map(d_accel_sig_us, ACCEL_SIG_MIN, ACCEL_SIG_MAX, (-1.)*DANGER_MAX_VEL, DANGER_MAX_VEL);
    }
    else if (slow == false) {
        if (i_turtlecar_gear == 1) {
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
    }

    m_turtle_cmd.linear.x = d_linear_speed;
    m_turtle_cmd.linear.y = 0.;
    m_turtle_cmd.linear.z = 0.;
    m_turtle_cmd.angular.x = 0.;
    m_turtle_cmd.angular.y = 0.;
    m_turtle_cmd.angular.z = d_angular_speed;

    m_wall_dist = GetWallDist(d_turtle_pose_x, d_turtle_pose_y);
    turtle_cmd_pub.publish(m_turtle_cmd);

}

double TurtleCar::WallDist() {
    return m_wall_dist;
}

// for debugging
void TurtleCar::PrintInfo() {
    std::cout <<
    "================================" << "\n" <<
        "motor driver output" << "\n\n" << 
        "    steer sig : " << m_steer_us << "\n" <<
        "    accel sig : " << m_accel_us << "\n" << 
        "    linear x   : " << m_turtle_cmd.linear.x << "\n" <<
        "    angular z  : " << m_turtle_cmd.linear.x << "\n" <<
    "    gear       : " << m_gear << "\n" <<
    "    wall dist  : " << m_wall_dist << "\n" <<
    std::endl;
}


// callback functions
void TurtleCar::SteeringCallback(const std_msgs::Int32::ConstPtr& msg) {
    m_steer_us = msg->data;
}
void TurtleCar::AccelCallback(const std_msgs::Int32::ConstPtr& msg) {
    m_accel_us = msg->data;
}
void TurtleCar::TurtlePoseCallback(const turtlesim_msgs::Pose::ConstPtr& msg) {
    m_turtle_pose_x = msg->x;
    m_turtle_pose_y = msg->y;
}

bool TurtleCar::GearShiftCallback(turtlecar::SetGearShift::Request &req, turtlecar::SetGearShift::Response &res) {
    res.gear_before = m_gear;
    m_gear = req.gear;
    res.gear_after = m_gear;

    ROS_INFO("\ngear input : %d\nsending gear status %d -> %d", (int)req.gear, (int)res.gear_before, (int)res.gear_after);

    return true;
}