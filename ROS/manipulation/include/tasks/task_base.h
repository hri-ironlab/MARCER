#ifndef TASKBASE_H
#define TASKBASE_H

#include <tasks/task_parameters.h>
#include <moveit/task_constructor/task.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <scene_graph/QuerySceneGraph.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>


using namespace moveit::task_constructor;

class TaskBase
{
public:
  TaskBase(const std::string &task_name, const ros::NodeHandle &nh);
  virtual ~TaskBase();
  virtual bool init(const TaskParameters &parameters) = 0;
  bool initTask();
  void addStageToTask(Stage::pointer &&stage);

  bool querySceneGraph(const std::string &node_name,
                       const std::string &relationship,
                       const std::string &attribute_name,
                       std::vector<std::string> &related_nodes,
                       std::vector<std::string> &attributes);

  // Associates a property with a specific group and name.
  void setProperty(const std::string &group, const std::string &name);

  // Makes specified properties available for serialization container.
  void exposeTo(SerialContainer &container, const std::initializer_list<std::string> &properties);

  // Retrieves names of link models with collision geometry for a specified group.
  std::vector<std::string> getLinkModelNamesWithCollisionGeometry(const std::string &group_name);

  // Retrieves the joint model group for a specified group name.
  const robot_model::JointModelGroup *getJointModelGroup(const std::string &group_name);

  double getHeightOffsetForSurface(const std::string &object_name, const std::string &place_surface_name, const double place_surface_offset);
  std::map<std::string, moveit_msgs::AttachedCollisionObject> getAttachedObjects();

  // Self collision functions
  bool applySelfCollisionAvoidance(bool add);
  moveit_msgs::CollisionObject getObjectMesh(const std::string &name, moveit_msgs::CollisionObject &collision_object);
  moveit_msgs::PlanningScene updatePlanningScene();  

  moveit_msgs::MoveItErrorCodes plan(int max_solutions = 1);
  void preempt();
  bool execute();

  void getSolutionMsg(moveit_task_constructor_msgs::Solution &solution);

  // Debugging output message with task name
  void TASK_INFO(const std::string &info);

protected:
  Stage *current_state_stage_;
  const std::string task_name_;
  std::string attached_object_name_;
private:
  TaskPtr task_;
  ros::NodeHandle nh_;
  ros::ServiceClient query_scene_graph_client;
};

#endif
