#include <tasks/task_parameters_loader.h>

constexpr char LOGNAME[] = "task_parameters_loader";

TaskParametersLoader::TaskParametersLoader() {}

void TaskParametersLoader::loadParameters(const ros::NodeHandle &pnh_)
{
    /****************************************************
     *                                                  *
     *               Load Parameters                    *
     *                                                  *
     ***************************************************/
    ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

    // Planning group properties
    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", parameters.arm_group_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name", parameters.hand_group_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", parameters.eef_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", parameters.hand_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "base_frame", parameters.base_frame_);
    
    std::string horizontal_grasp_frame_transform_name;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "horizontal_grasp_frame_transform_name", horizontal_grasp_frame_transform_name);
    Eigen::Isometry3d horizontal_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "horizontal_grasp_frame_transform", horizontal_grasp_frame_transform);
    parameters.grasp_frame_transforms_[horizontal_grasp_frame_transform_name] = horizontal_grasp_frame_transform;
    parameters.grasp_frame_transforms_insertion_order_.push_back(horizontal_grasp_frame_transform_name);

    std::string vertical_grasp_frame_transform_name;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "vertical_grasp_frame_transform_name", vertical_grasp_frame_transform_name);
    Eigen::Isometry3d vertical_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "vertical_grasp_frame_transform", vertical_grasp_frame_transform);
    parameters.grasp_frame_transforms_[vertical_grasp_frame_transform_name] = vertical_grasp_frame_transform;
    parameters.grasp_frame_transforms_insertion_order_.push_back(vertical_grasp_frame_transform_name);

    std::string diagonal_grasp_frame_transform_name;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "diagonal_grasp_frame_transform_name", diagonal_grasp_frame_transform_name);
    Eigen::Isometry3d diagonal_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "diagonal_grasp_frame_transform", diagonal_grasp_frame_transform);
    parameters.grasp_frame_transforms_[diagonal_grasp_frame_transform_name] = diagonal_grasp_frame_transform;
    parameters.grasp_frame_transforms_insertion_order_.push_back(diagonal_grasp_frame_transform_name);

    std::string long_horizontal_grasp_frame_transform_name;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "long_horizontal_grasp_frame_transform_name", long_horizontal_grasp_frame_transform_name);
    Eigen::Isometry3d long_horizontal_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "long_horizontal_grasp_frame_transform", long_horizontal_grasp_frame_transform);
    parameters.grasp_frame_transforms_[long_horizontal_grasp_frame_transform_name] = long_horizontal_grasp_frame_transform;
    parameters.grasp_frame_transforms_insertion_order_.push_back(long_horizontal_grasp_frame_transform_name);

    // std::string long_vertical_grasp_frame_transform_name;
    // errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "long_vertical_grasp_frame_transform_name", long_vertical_grasp_frame_transform_name);
    // Eigen::Isometry3d long_vertical_grasp_frame_transform;
    // errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "long_vertical_grasp_frame_transform", long_vertical_grasp_frame_transform);
    // parameters.grasp_frame_transforms_[long_vertical_grasp_frame_transform_name] = long_vertical_grasp_frame_transform;
    // parameters.grasp_frame_transforms_insertion_order_.push_back(long_vertical_grasp_frame_transform_name);
    
    // Predefined pose targets
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose", parameters.hand_open_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose", parameters.hand_close_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_ready_pose", parameters.arm_ready_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_tuck_pose", parameters.arm_tuck_pose_);

    // Pick/Place metrics
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", parameters.approach_object_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", parameters.approach_object_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", parameters.lift_object_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", parameters.lift_object_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", parameters.place_surface_offset_);

    // Could be used in the future, not used right now. Included because bottle was being placed sideways
    moveit_msgs::Constraints upright_constraint;
    upright_constraint.name = "upright_constraint";
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
    parameters.constraints_[upright_constraint.name] = upright_constraint;

    moveit_msgs::Constraints downward_constraint;
    upright_constraint.name = "downward_constraint";
    upright_constraint.orientation_constraints.resize(1);
    {
        moveit_msgs::OrientationConstraint &c = upright_constraint.orientation_constraints[0];
        c.link_name = parameters.hand_frame_;       // constraining hand frame
        c.header.frame_id = parameters.base_frame_; // reference the base frame
        c.orientation.w = -1.0;
        c.absolute_y_axis_tolerance = 0.65;
        c.absolute_z_axis_tolerance = 0.65;
        c.absolute_x_axis_tolerance = M_PI;
        c.weight = 1.0;
    }
    parameters.constraints_[downward_constraint.name] = downward_constraint;

    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

TaskParameters &
TaskParametersLoader::getParameters()
{
    return parameters;
}
