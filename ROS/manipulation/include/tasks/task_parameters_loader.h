#ifndef TASK_PARAMETERS_LOADER_H
#define TASK_PARAMETERS_LOADER_H

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotState.h>
#include <tasks/task_parameters.h>

class TaskParametersLoader
{
public:
    TaskParametersLoader();
    void loadParameters(const ros::NodeHandle &pnh);
    TaskParameters &getParameters();

private:
    TaskParameters parameters;
};

#endif
