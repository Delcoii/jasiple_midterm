#ifndef __SAFETY_PLAN_H__
#define __SAFETY_PLAN_H__


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "turtlecar/WallDetectAction.h"


class SafetyPlanAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<turtlecar::WallDetectAction> as_;
    std::string action_name_;
    turtlecar::WallDetectFeedback feedback_;
    turtlecar::WallDetectResult result_;

    double wall_dist_;

public:
    SafetyPlanAction(std::string name)
        :
        as_(nh_, name, boost::bind(&SafetyPlanAction::executeCB, this, _1), false),
        action_name_(name),
        wall_dist_(999.)
    {
        as_.start();
    }

    ~SafetyPlanAction(void) {}

    void SetWallDist(double dist);
    void executeCB(const turtlecar::WallDetectGoalConstPtr& goal);
};

void SafetyPlanAction::SetWallDist(double dist) {
    wall_dist_ = dist;
}


void SafetyPlanAction::executeCB(const turtlecar::WallDetectGoalConstPtr& goal) {
    ros::Rate action_loop(60.);
    bool success = false;

    if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s : Preempted", action_name_.c_str());

        as_.setPreempted();
        success = false;
        return;
    }

    while (ros::ok()) {
        feedback_.wall_distance = wall_dist_;


        as_.publishFeedback(feedback_);

        if (goal->danger_distance > wall_dist_) {
            success = true;
            break;
        }
        action_loop.sleep();
    }


    if (success) {
        result_.dangerous = true;
        ROS_INFO("%s: Wall is so close!", action_name_.c_str());
        as_.setSucceeded(result_);
    }

}




#endif // __SAFETY_PLAN_H__