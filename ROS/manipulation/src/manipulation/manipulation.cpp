#include <manipulation/manipulation.h>

constexpr char LOGNAME[] = "moveit_task_constructor";
using namespace manipulation;

Manipulation::Manipulation(const ros::NodeHandle &nh) : nh_(nh)
{
}

Manipulation::~Manipulation()
{
  for (auto &task : task_plans)
  {
    task.second.reset();
  }
}

void Manipulation::setParameters(TaskParameters &params)
{
  parameters = params;
}

// Service handler that receives message from executive to plan a task
bool Manipulation::handleManipulationPlanRequest(manipulation::GetManipulationPlan::Request &req, manipulation::GetManipulationPlan::Response &res)
{
  ROS_INFO("here1");
  if(req.manipulation_plan_request.task_type == ManipulationPlanRequest::DONE)
  {
  	ROS_INFO("here2");
    // Iterate through the map and reset each unique_ptr
    for (auto& pair : task_plans) {
        pair.second.reset(); // Reset the unique_ptr
    }
    // Clear the map
    task_plans.clear();
    ROS_INFO("here3");
    res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }

  std_srvs::Trigger srv;

  // ROS_INFO("[manipulation_node] Request: Update Planning Scene");

  // if (update_planning_scene_service.call(srv)) {
  //     ROS_INFO("[manipulation_node] Response: Planning Scene updated");
  // } else {
  //     res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  //     ROS_ERROR("[manipulation_node] Response: Planning Scene failed to update for %s", req.manipulation_plan_request.task_name.c_str());
  //     return true;
  // }
  ROS_INFO("here4");
  parameters.place_pose_ = req.manipulation_plan_request.place_pose;

  // PICK, PLACE, MOVE_ARM, OPEN_GRIPPER, CLOSE_GRIPPER, TODO: SHAKE, POUR, PUSH, PULL, OPEN_DOOR, CLOSE_DOOR
  parameters.task_type_ = req.manipulation_plan_request.task_type;

  // Use task name to label it within the list of tasks
  std::string task_name = req.manipulation_plan_request.task_name;

  // Provide place surface
  parameters.target_object_name_ = req.manipulation_plan_request.target_object_name;

    ROS_INFO("here5");
  if(task_plans.find(task_name) != task_plans.end())
  {
    task_plans[task_name].reset();
  }

    ROS_INFO("here6");
  task_plans[task_name] = TaskFactory::createTask(parameters.task_type_, task_name.c_str(), nh_);

  // Return failure if initialization fails
  if (!task_plans[task_name]->init(parameters))
  {
    res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return true;
  }
  // Get if planning suceeded or failed
  res.manipulation_plan_response.error_code = task_plans[task_name]->plan();

  // Only add the solution if planning succeeded
  if (res.manipulation_plan_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    task_plans[task_name]->getSolutionMsg(res.manipulation_plan_response.solution);
    task_solution_publisher.publish(res.manipulation_plan_response.solution);
  }

  return true;
}
