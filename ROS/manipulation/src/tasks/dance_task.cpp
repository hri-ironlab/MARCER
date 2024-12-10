#include <tasks/dance_task.h>

const bool registered = TaskFactory::registerTask(
    manipulation::ManipulationPlanRequest::DANCE,
    [](const std::string &taskName, const ros::NodeHandle &nh) -> std::unique_ptr<TaskBase>
    {
      return std::make_unique<DanceTask>(taskName, nh);
    });

using namespace manipulation;

DanceTask::DanceTask(const std::string &task_name, const ros::NodeHandle &nh) : TaskBase(task_name, nh)
{
  current_state_stage_ = nullptr;
}

bool DanceTask::init(const TaskParameters &parameters)
{

  TASK_INFO("Initializing mtc pipeline");
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(.2);
  cartesian_planner->setMaxAccelerationScaling(.2);
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
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(20);

    // Verify that object is not attached for picking and if object is attached for placing
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase &s, std::string &comment)
                                       {
      s.start()->scene()->printKnownObjects(std::cout);

        if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.target_object_name_))
        {
          comment = "object with id '" + parameters.target_object_name_ + "' is attached and must be placed down first.";
          return false;
        }
      return true; });

    current_state_stage_ = applicability_filter.get();
    addStageToTask(std::move(applicability_filter));
  }
  {
    // Open hand stage
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_open_pose_);
      addStageToTask(std::move(stage));
    }

    std::vector<std::string> joint_names{
        "torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    std::vector<std::vector<double>> disco_poses{
        {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0},
        {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
        {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
        {0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0},
        {0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0},
        {0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0},
        {0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0}};

    int i = 1;
    for (const auto &pose : disco_poses)
    {
      moveit_msgs::RobotState robot_state;
      robot_state.is_diff = true;

      robot_state.joint_state.name = joint_names;
      robot_state.joint_state.position = pose;

      std::string goal = "Dance " + std::to_string(i);
      i++;
      auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
      stage->setGroup(parameters.arm_group_name_);
      stage->setGoal(robot_state);
      addStageToTask(std::move(stage));
    }
  }
  return initTask();
}
