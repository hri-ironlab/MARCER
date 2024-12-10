#ifndef TASKFACTORY_H
#define TASKFACTORY_H

#pragma once
#include "tasks/task_base.h"
#include "manipulation/GetManipulationPlan.h"
#include <unordered_map>
#include <functional>

// TaskFactory: Manages and creates TaskBase instances.
class TaskFactory
{
public:
    // CreatorFunction: Function type for creating TaskBase instances.
    // using CreatorFunction = std::function<TaskBase*(const std::string&)>;
    using CreatorFunction = std::function<std::unique_ptr<TaskBase>(const std::string &, const ros::NodeHandle &)>;

    // Registers a task type from GetManipulationPlan.msg with its creator function.
    static bool registerTask(const int &task, CreatorFunction creator);

    // Creates a TaskBase instance based on the task type and name.
    static std::unique_ptr<TaskBase> createTask(const int &task, const std::string &taskName, const ros::NodeHandle &nh);

private:
    // Unordered map to access the Task object factory
    static std::unordered_map<int, CreatorFunction> &getRegistry();
};

#endif
