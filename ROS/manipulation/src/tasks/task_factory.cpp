#include "tasks/task_factory.h"

// Function to access the static registry of task creator functions
std::unordered_map<int, TaskFactory::CreatorFunction> &TaskFactory::getRegistry()
{
    // Static local variable to ensure the persistence of the registry across function calls
    static std::unordered_map<int, CreatorFunction> registry;
    return registry;
}

// Function to register a task with its corresponding creator function
bool TaskFactory::registerTask(const int &taskType, CreatorFunction creator)
{
    // Access the registry using getRegistry(), and associate the task ID with the creator function
    getRegistry()[taskType] = creator;
    return true;
}

std::unique_ptr<TaskBase> TaskFactory::createTask(const int &taskType, const std::string &taskName, const ros::NodeHandle &nh)
{
    // Find the creator function associated with the task type from ManipulationPlanRequest.msg in the registry
    auto it = getRegistry().find(taskType);

    // Check if the creator function is found
    if (it != getRegistry().end())
    {
        // Call the creator function with the task name and return the created task wrapped in a unique_ptr
        return std::unique_ptr<TaskBase>((it->second)(taskName, nh));
    }

    // Return nullptr if the task ID is not registered
    return nullptr;
}
