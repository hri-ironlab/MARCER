#include <tasks/pour_task.h>

const bool registered = TaskFactory::registerTask(
    manipulation::ManipulationPlanRequest::POUR,
    [](const std::string &taskName, const ros::NodeHandle &nh) -> std::unique_ptr<TaskBase>
    {
      return std::make_unique<PourTask>(taskName, nh);
    });

using namespace moveit::task_constructor;

PourTask::PourTask(const std::string &task_name, const ros::NodeHandle &nh) : TaskBase(task_name, nh)
{
  current_state_stage_ = nullptr;
}

bool PourTask::init(const TaskParameters &parameters)
{
  TASK_INFO("Initializing mtc pipeline");
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(.05);
  cartesian_planner->setMaxAccelerationScaling(.05);
  cartesian_planner->setStepSize(.01);

  // Set task properties, names used for the specific arm group by robot
  setProperty("group", parameters.arm_group_name_);
  setProperty("eef", parameters.eef_name_);
  setProperty("hand", parameters.hand_group_name_);
  setProperty("ik_frame", parameters.hand_frame_);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  attached_object_name_ = getAttachedObjects().begin()->second.object.id;
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(20);

    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase &s, std::string &comment)
                                       {
        if (attached_object_name_.empty())
        {
          comment = "Object is not attached to be used for placing";
          return false;
        }

      return true; });
    current_state_stage_ = applicability_filter.get();
    addStageToTask(std::move(applicability_filter));
  }
  /****************************************************
    .... *            Pour Start     **************** *
  ***************************************************/

  moveit_msgs::Constraints upright_constraint;
  upright_constraint.name = "pouring_hand_constraints";
  upright_constraint.orientation_constraints.resize(1);
  {
    moveit_msgs::OrientationConstraint &c = upright_constraint.orientation_constraints[0];
    c.link_name = parameters.hand_frame_;       // constraining hand frame
    c.header.frame_id = parameters.base_frame_; // reference the base frame
    c.orientation.w = 1.0;

    c.absolute_y_axis_tolerance = 0.65;
    c.absolute_z_axis_tolerance = 0.65;
    c.absolute_x_axis_tolerance = M_PI;

    c.weight = 1.0;
  }
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to pre-pour pose",
        stages::Connect::GroupPlannerVector{{parameters.arm_group_name_, sampling_planner}});
    stage->setTimeout(20.0);
    stage->setPathConstraints(upright_constraint);
    stage->properties().configureInitFrom(Stage::PARENT); // TODO: convenience-wrapper
    addStageToTask(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::GeneratePose>("pose above object to pour into");
    stage->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"}); //
    geometry_msgs::PoseStamped p;
    p.header.frame_id = parameters.target_object_name_;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;
    p.pose.position.z = .3;
    stage->setPose(p);
    stage->properties().configureInitFrom(Stage::PARENT);

    stage->setMonitoredStage(current_state_stage_);

    auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage));
    wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"}); //
    wrapper->setMaxIKSolutions(8);
    wrapper->setIKFrame(parameters.grasp_frame_transforms_.at("horizontal"), parameters.hand_frame_);
    // TODO adding this will initialize "target_pose" which is internal (or
    // isn't it?)
    // wrapper->properties().configureInitFrom(Stage::PARENT);
    wrapper->properties().property("eef").configureInitFrom(Stage::PARENT, "eef");     //
    wrapper->properties().property("group").configureInitFrom(Stage::PARENT, "group"); //
    wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
    addStageToTask(std::move(wrapper));
  }
  {
    auto stage = std::make_unique<manipulation_stages::PourInto>("pouring");
    stage->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"}); //
    stage->properties().property("group").configureInitFrom(Stage::PARENT, "group");            //
    stage->setBottle(attached_object_name_);
    stage->setContainer(parameters.target_object_name_);
    stage->setPourOffset(Eigen::Vector3d(0, 0.02, 0.01));
    stage->setTiltAngle(2.0);
    stage->setPourDuration(ros::Duration(4.0));
    {
      geometry_msgs::Vector3Stamped pouring_axis;
      pouring_axis.header.frame_id = parameters.target_object_name_;
      pouring_axis.vector.x = 1.0;
      stage->setPouringAxis(pouring_axis);
    }
    stage->properties().configureInitFrom(Stage::PARENT);
    addStageToTask(std::move(stage));
  }
  return initTask();
}
