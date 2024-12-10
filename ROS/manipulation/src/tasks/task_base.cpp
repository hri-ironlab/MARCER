#include <tasks/task_base.h>

// Initializes the TaskBase with a specified task name.
TaskBase::TaskBase(const std::string &task_name, const ros::NodeHandle &nh) : task_name_(task_name), nh_(nh)
{
    query_scene_graph_client = nh_.serviceClient<scene_graph::QuerySceneGraph>("/scene_graph/query");
    // Create a new Task instance and load the robot model associated with it.
    task_.reset(new moveit::task_constructor::Task());
    // Setting this names the stages the task, not the solution which is used
    // to publish the solution on topic
    task_->stages()->setName(task_name);
    task_->loadRobotModel();
}

TaskBase::~TaskBase()
{
}

bool TaskBase::applySelfCollisionAvoidance(bool add)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    if(add)
    {
        moveit_msgs::CollisionObject keepout_object;
        keepout_object.id = "keepout";
        keepout_object.header.frame_id = "base_link";
        keepout_object.pose.position.z = 0.375;
        keepout_object.pose.orientation.w = 1.0;
        keepout_object.operation = moveit_msgs::CollisionObject::ADD;
        getObjectMesh(keepout_object.id, keepout_object);
        collision_objects.push_back(keepout_object);

        moveit_msgs::CollisionObject ground_object;
        ground_object.id = "ground";
        ground_object.header.frame_id = "base_link";
        ground_object.pose.position.z = -0.03;
        ground_object.pose.orientation.w = 1.0;
        ground_object.operation = moveit_msgs::CollisionObject::ADD;
        getObjectMesh(ground_object.id, ground_object);
        collision_objects.push_back(ground_object);

        moveit_msgs::CollisionObject head_camera_object;
        head_camera_object.id = "head_camera";
        head_camera_object.header.frame_id = "head_pan_link";
        head_camera_object.pose.position.z = 0.2;
        head_camera_object.pose.orientation.w = 1.0;
        head_camera_object.primitives.resize(1);
        head_camera_object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
        head_camera_object.primitives[0].dimensions.resize(3);
        head_camera_object.primitives[0].dimensions[0] = 0.25;
        head_camera_object.primitives[0].dimensions[1] = 0.3;
        head_camera_object.primitives[0].dimensions[2] = 0.15;
        head_camera_object.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.push_back(head_camera_object);
    }
    else
    {        
        moveit_msgs::CollisionObject keepout_object;
        keepout_object.id = "keepout";
        keepout_object.operation = moveit_msgs::CollisionObject::REMOVE;
        collision_objects.push_back(keepout_object);

        moveit_msgs::CollisionObject ground_object;
        ground_object.id = "ground";
        ground_object.operation = moveit_msgs::CollisionObject::REMOVE;
        collision_objects.push_back(ground_object);

        moveit_msgs::CollisionObject head_camera_object;
        head_camera_object.id = "head_camera";
        head_camera_object.operation = moveit_msgs::CollisionObject::REMOVE;
        collision_objects.push_back(head_camera_object);
    }
    return psi.applyCollisionObjects(collision_objects);
}

moveit_msgs::CollisionObject
TaskBase::getObjectMesh(const std::string &name, moveit_msgs::CollisionObject &collision_object)
{
    // Load meshes of objects into the scene
    collision_object.id = name;

    std::string resource = "package://manipulation/meshes/" + name + ".stl";

    // load mesh
    const Eigen::Vector3d scaling(1, 1, 1);
    shapes::Shape *shape = shapes::createMeshFromResource(resource, scaling);

    if (!shape)
    {
        ROS_WARN_STREAM("[WorldMonitor] Unable to load mesh for object: " << name);
        return collision_object;
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(shape, shape_msg);

    collision_object.meshes.resize(1);
    collision_object.meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);

    collision_object.mesh_poses.resize(1);
    collision_object.mesh_poses[0].orientation.w = 1.0;
    return collision_object;
}

double
TaskBase::getHeightOffsetForSurface(const std::string &object_name, const std::string &place_surface_name, const double place_surface_offset)
{
    moveit_msgs::CollisionObject object_co = getAttachedObjects()[object_name].object;

    double object_place_offset = 0.0;

    // Return an empty message if object not attached
    if (object_co.primitives.empty() && object_co.meshes.empty())
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Object with id '" << object_name << "' is not attached, so it cannot be placed");
        return 0;
    }

    // Get the offset of the object
    if (!object_co.meshes.empty())
    {
        double x, y, z;
        geometric_shapes::getShapeExtents(object_co.meshes[0], x, y, z);
        object_place_offset += z / 2 + place_surface_offset;
    }
    else if (object_co.primitives[0].type == shape_msgs::SolidPrimitive::BOX)
    {
        object_place_offset += 0.5 * object_co.primitives[0].dimensions[2] + place_surface_offset;
    }
    else
    {
        object_place_offset += 0.5 * object_co.primitives[0].dimensions[0] + place_surface_offset;
    }

    return object_place_offset;
}

// Executes the planned task trajectory.
bool TaskBase::execute()
{
    TASK_INFO("Executing solution trajectory");

    // Publish the task solution for introspection.
    task_->introspection().publishSolution(*task_->solutions().front());

    actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>

    execute("execute_task_solution", true);
    execute.waitForServer();

    moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
    getSolutionMsg(execute_goal.solution);
    execute.sendGoalAndWait(execute_goal);

    moveit_msgs::MoveItErrorCodes execute_result = execute.getResult()->error_code;

    // Handle the execution result.
    if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Task execution failed and returned: " << execute_result.val);
        return false;
    }
    return true;
}

void TaskBase::getSolutionMsg(moveit_task_constructor_msgs::Solution &solution)
{
    
    task_->solutions().front()->fillMessage(solution);
}

void TaskBase::preempt()
{
    task_->preempt();
}

moveit_msgs::MoveItErrorCodes TaskBase::plan(int max_solutions)
{
    TASK_INFO("Searching for task solutions");

    moveit_msgs::MoveItErrorCodes error_code = task_->plan(max_solutions);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        // Publish the task solution for external consumption.
        task_->introspection().publishSolution(*task_->solutions().front());
        TASK_INFO("Planning succeeded");
    }
    else
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Planning failed and returned: " << error_code.val);
    }

    return error_code;
}

// Makes specified properties available for serialization container.
void TaskBase::exposeTo(SerialContainer &container, const std::initializer_list<std::string> &properties)
{
    task_->properties().exposeTo(container.properties(), properties);
}

// Associates a property with a specific group and name.
void TaskBase::setProperty(const std::string &group, const std::string &name)
{
    task_->setProperty(group, name);
}

// Appends a stage to the task execution pipeline.
void TaskBase::addStageToTask(Stage::pointer &&stage)
{
    task_->add(std::move(stage));
}

// Retrieves the joint model group for a specified group name.
const robot_model::JointModelGroup *TaskBase::getJointModelGroup(const std::string &group_name)
{
    return task_->getRobotModel()->getJointModelGroup(group_name);
}

// Retrieves names of link models with collision geometry for a specified group.
std::vector<std::string> TaskBase::getLinkModelNamesWithCollisionGeometry(const std::string &group_name)
{
    return task_->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNamesWithCollisionGeometry();
}

/** Get attached objects */
std::map<std::string, moveit_msgs::AttachedCollisionObject> 
TaskBase::getAttachedObjects()	
{
    moveit::planning_interface::PlanningSceneInterface psi;
    return psi.getAttachedObjects();
}

// Outputs informational messages related to the task.
void TaskBase::TASK_INFO(const std::string &info)
{
    ROS_INFO("[%s]: %s", task_name_.c_str(), info.c_str());
}

bool TaskBase::initTask()
{
    applySelfCollisionAvoidance(true);
    sleep(1);
    try
    {
        // Attempt to initialize the task.
        task_->init();
    }
    catch (InitStageException &e)
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Initialization failed: " << e);
        return false;
    }
    applySelfCollisionAvoidance(false);
    sleep(1);
    return true;
}

// Function to query the scene graph for support surface and subframes
bool TaskBase::querySceneGraph(const std::string &node_name,
                               const std::string &relationship_type,
                               const std::string &attribute_name,
                               std::vector<std::string> &related_nodes,
                               std::vector<std::string> &attributes)
{
    // Query for support surface
    scene_graph::QuerySceneGraph srv;
    srv.request.node_name = node_name;
    srv.request.relationship_type = relationship_type;
    srv.request.attribute_name = attribute_name;

    if (query_scene_graph_client.call(srv))
    {
        related_nodes = srv.response.related_nodes;
        attributes = srv.response.attributes;
        return true;
    }

    return false;
}