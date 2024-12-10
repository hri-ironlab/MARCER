#ifndef MANIPULATION_H
#define MANIPULATION_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/ApplyPlanningScene.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <moveit_task_constructor_msgs/Solution.h>
#include <actionlib/server/simple_action_server.h>
#include <tasks/task_parameters.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <manipulation/GetManipulationPlan.h>
#include <manipulation/ManipulationPlanRequest.h>
#include <manipulation/ManipulationPlanResponse.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseArray.h>
#include <scene_graph/QuerySceneGraph.h>

#include "tasks/task_base.h"
#include "tasks/task_factory.h"
#include "tasks/pick_task.h"
#include "tasks/place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/close_hand_task.h"
#include "tasks/open_hand_task.h"
#include "tasks/wave_task.h"
#include "tasks/pour_task.h"
#include "tasks/dance_task.h"

using namespace moveit::task_constructor;

class Manipulation
{
public:
  Manipulation(const ros::NodeHandle &nh);
  ~Manipulation();

  void setParameters(TaskParameters &params);
  bool handleManipulationPlanRequest(manipulation::GetManipulationPlan::Request &req, manipulation::GetManipulationPlan::Response &res);

  ros::ServiceServer get_manipulation_plan_service;
  ros::ServiceClient update_planning_scene_service;
  ros::Publisher task_solution_publisher; 
private:
  TaskParameters parameters;
  ros::NodeHandle nh_;
  std::unordered_map<std::string, std::unique_ptr<TaskBase>> task_plans;
};
#endif
