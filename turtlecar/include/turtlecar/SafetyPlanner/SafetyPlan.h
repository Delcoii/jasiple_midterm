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

public:
    SafetyPlanAction(std::string name)
        : as_(nh_, name, boost::bind(&SafetyPlanAction::executeCB, this, _1), false),
        
        action_name_(name)
    {
        as_.start();
    }

    ~SafetyPlanAction(void) {}

    void executeCB(const turtlecar::WallDetectGoalConstPtr& goal);
};

void SafetyPlanAction::executeCB(const turtlecar::WallDetectGoalConstPtr& goal) {
    return;
}

#endif // __SAFETY_PLAN_H__