#include <iostream>
#include "manipulation/manipulation.h"
#include "tasks/task_parameters_loader.h"
#include "manipulation/ManipulationPlanRequest.h"
#include "manipulation/GetManipulationPlan.h"
#include "actionlib/client/simple_action_client.h"
#include "moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h"
#include "moveit_task_constructor_msgs/ExecuteTaskSolutionGoal.h"
#include "ros/ros.h"

constexpr char LOGNAME[] = "manipulation_request";

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manipulation_request");
    ros::NodeHandle nh, pnh("~");

    // Create a client for the ManipulationPlanRequest service
    ros::ServiceClient client = nh.serviceClient<manipulation::GetManipulationPlan>("get_manipulation_plan");

    // Wait for the service to become available
    client.waitForExistence();

    // Create a service request
    manipulation::GetManipulationPlan srv;
    srv.request.manipulation_plan_request.task_type = manipulation::ManipulationPlanRequest::PICK;
    srv.request.manipulation_plan_request.task_name = "pick";
    srv.request.manipulation_plan_request.target_object_name = "Bottle";
    // Fill in the request fields as needed

    // Call the service
    if (client.call(srv))
    {
        // Service call succeeded
        ROS_INFO_STREAM(LOGNAME << ": Service call successful");
        // Process the response if needed
    }
    else
    {
        // Service call failed
        ROS_ERROR_STREAM(LOGNAME << ": Failed to call service ManipulationPlanRequest");
    }
    ros::spin();
    return 0;
}
