#include <world_monitor/world_monitor.h>

WorldMonitor::WorldMonitor()
{
}

void WorldMonitor::objectDetectionsCallback(const vision_msgs::Detection3DArray::ConstPtr &msg)
{
    object_detections = *msg;
}

void WorldMonitor::visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr &msg)
{
    // Assuming all manipulable objects are known, add them to object_names array
    if (msg->method == "parameter server" && objects_info.empty())
    {
        ros::param::get(msg->database_location, objects_info);
    }
}

bool WorldMonitor::getSceneObjectsRequest(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res)
{
    if (!active_scene_objects.collision_objects.empty())
    {
        res.scene.world = active_scene_objects;
        return true;
    }
    else
    {
        return false;
    }
}

bool WorldMonitor::getAttachedObjectRequest(world_monitor::GetAttachedObject::Request &req, world_monitor::GetAttachedObject::Response &res)
{
    // Retrieve the name of the first attached object
    res.attached_object_name = GetAttachedObject();
    return true; 
}

std::string WorldMonitor::GetAttachedObject()
{
    std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = 
    planning_scene_interface.getAttachedObjects();

    // Check if there are attached objects
    if (attached_objects.empty()) {
        return ""; 
    }
    return attached_objects.begin()->first;
}

bool WorldMonitor::updatePlanningSceneRequest(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Service call /update_planning_scene will move object positions
    moveit_msgs::PlanningScene planning_scene = updatePlanningScene();

    if (applyPlanningScene(planning_scene))
    {
        res.success = true;
        res.message = "Planning Scene Updated";
        return true;
    }
    else
    {
        res.success = false;
        res.message = "Failed to update planning scene";
        return false;
    }
}

moveit_msgs::PlanningScene
WorldMonitor::updatePlanningScene()
{
    // Update planning scene based on what is viewed
    moveit_msgs::PlanningScene planning_scene;
    std::string attached_object = GetAttachedObject();
    for (const auto &detection : object_detections.detections)
    {
        if(attached_object == objects_info[detection.results[0].id])
        {
            for (const auto &collision_object : active_scene_objects.collision_objects)
            {
                // Update position of object picked by robot
                if (collision_object.id == attached_object)
                {
                    moveit_msgs::AttachedCollisionObject attached_collision_object;
                    attached_collision_object.link_name = "gripper_link";
                    attached_collision_object.object.header.frame_id = "gripper_link";
                    attached_collision_object.object = collision_object;
                    attached_collision_object.object.pose = detection.bbox.center;
                    attached_collision_object.object.operation = moveit_msgs::CollisionObject::ADD;  // Corrected the operation to use the full enum path
                    // Set or append the attached collision object
                    planning_scene.robot_state.attached_collision_objects.clear();  // Clear to ensure only one attached object (if needed)
                    planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
                    break;
                }
            }
        }
        else{
            moveit_msgs::CollisionObject collision_object;
            collision_object.id = objects_info[detection.results[0].id];
            collision_object.header = detection.header;
            collision_object.pose = detection.bbox.center;
            collision_object.operation = moveit_msgs::CollisionObject::MOVE;
            planning_scene.world.collision_objects.push_back(collision_object);
        }
    }

    planning_scene.is_diff = true;
    return planning_scene;
}

bool WorldMonitor::applyPlanningScene(moveit_msgs::PlanningScene &planning_scene)
{
    // Apply Planning scene for initial add and continued update
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    if (planning_scene_client.call(srv))
    {
        ROS_INFO("[world_monitor_node] Planning Scene applied.");
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("[world_monitor_node] Failed to call Planning Scene service");
        return false;
    }
}

bool WorldMonitor::initializePlanningScene()
{
    // Don't initialize the planning scene until we have everything we need
    if (object_detections.detections.empty() || objects_info.empty() || scene_object_properties.collision_objects.empty())
    {
        return true;
    }
    moveit_msgs::PlanningScene planning_scene;
    for (auto detection : object_detections.detections)
    {
        for (auto collision_object : scene_object_properties.collision_objects)
        {
            if (collision_object.id == objects_info[detection.results[0].id])
            {
                // Get mesh file and add to object
                if (collision_object.type.key == "mesh")
                {
                    getObjectMesh(collision_object.id, collision_object);
                    collision_object.header = detection.header;
                }
                else if (collision_object.type.key == "surface")
                {
                    // Surface objects are colored blue
                    moveit_msgs::ObjectColor color;
                    color.id = collision_object.id;
                    color.color.b = 1.0;
                    color.color.a = 1.0;
                    planning_scene.object_colors.push_back(color);
                }
                else if(collision_object.type.key == "collision_object")
                {
                    continue;
                }
                collision_object.pose = detection.bbox.center;
                planning_scene.world.collision_objects.push_back(collision_object);
                active_scene_objects.collision_objects.push_back(collision_object);
                addObjectTransform(collision_object);
                ROS_INFO_STREAM("[world_monitor_node] Adding object to scene: " << collision_object.id);
                break;
            }
        }
    }

    for(auto surface : surface_object_properties.collision_objects)
    {
        // Surface objects are colored blue
        moveit_msgs::ObjectColor color;
        color.id = surface.id;
        color.color.b = 1.0;
        color.color.a = 1.0;
        planning_scene.object_colors.push_back(color);
        planning_scene.world.collision_objects.push_back(surface);
        // if(surface.type.key != "collision_object")
        // {
            active_scene_objects.collision_objects.push_back(surface);
        // }
        addObjectTransform(surface);
        ROS_INFO_STREAM("[world_monitor_node] Adding surface to scene: " << surface.id);
    }

    planning_scene.is_diff = true;

    if (applyPlanningScene(planning_scene))
    {
        ROS_INFO("[world_monitor_node] Planning Scene Initialized");
        return false;
    }
    else
    {
        ROS_ERROR_STREAM("[world_monitor_node] Service call failed");
        return true;
    }
}

void WorldMonitor::addObjectTransform(const moveit_msgs::CollisionObject &collisionObject)
{
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = collisionObject.header.frame_id;
    transform.child_frame_id = collisionObject.id;
    transform.transform.translation.x = collisionObject.pose.position.x;
    transform.transform.translation.y = collisionObject.pose.position.y;
    transform.transform.translation.z = collisionObject.pose.position.z;

    transform.transform.rotation.x = collisionObject.pose.orientation.x;
    transform.transform.rotation.y = collisionObject.pose.orientation.y;
    transform.transform.rotation.z = collisionObject.pose.orientation.z;
    transform.transform.rotation.w = collisionObject.pose.orientation.w;

    transformStampedArray.push_back(transform);

    for (int i = 0; i < collisionObject.subframe_names.size(); i++)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = collisionObject.id;
        transform.child_frame_id = collisionObject.subframe_names[i];
        transform.transform.translation.x = collisionObject.subframe_poses[i].position.x;
        transform.transform.translation.y = collisionObject.subframe_poses[i].position.y;
        transform.transform.translation.z = collisionObject.subframe_poses[i].position.z;
        transform.transform.rotation = collisionObject.subframe_poses[i].orientation;
        transformStampedArray.push_back(transform);
    }
}

void WorldMonitor::broadcastTransforms()
{
    for (auto transform : transformStampedArray)
    {
        transform.header.stamp = ros::Time::now();
        tfb.sendTransform(transform);
    }
}

moveit_msgs::CollisionObject
WorldMonitor::getObjectMesh(const std::string &name, moveit_msgs::CollisionObject &collision_object)
{
    // Load meshes of objects into the scene
    collision_object.id = name;

    std::string resource = "package://world_monitor/meshes/" + name + ".stl";

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

void WorldMonitor::loadObjectParameters(const std::string objects_filepath)
{
    // Read configuration from YAML file
    YAML::Node config = YAML::LoadFile(objects_filepath);

    // Iterate over each entry in the YAML file
    for (const auto &entry : config)
    {
        // Create a CollisionObject for each entry
        moveit_msgs::CollisionObject collisionObject;

        // Safety checks for required fields
        if (!entry["object_id"] || !entry["header_frame_id"] || !entry["primitive_types"] ||
            !entry["primitive_dimensions"] || !entry["primitive_poses"] || !entry["subframe_names"] || !entry["subframe_poses"])
        {
            // Handle missing fields or incorrect YAML structure
            ROS_ERROR_STREAM("Invalid YAML structure in the file.");
            continue;
        }

        collisionObject.id = entry["object_id"].as<std::string>();
        collisionObject.header.frame_id = entry["header_frame_id"].as<std::string>();
        collisionObject.type.key = entry["key"].as<std::string>();

        // Get information arrays from the entry
        const auto &primitiveTypes = entry["primitive_types"];
        const auto &primitiveDimensions = entry["primitive_dimensions"];
        const auto &primitivePoses = entry["primitive_poses"];

        const auto &subframeNames = entry["subframe_names"].as<std::vector<std::string>>();
        const auto &subframePoses = entry["subframe_poses"];

        const auto &meshPoses = entry["mesh_poses"];

        // Primitives
        for (std::size_t i = 0; i < primitiveTypes.size(); ++i)
        {
            // Assuming there is only one type and pose per primitive
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitiveTypes[i][0].as<int>();
            primitive.dimensions = primitiveDimensions[i].as<std::vector<double>>();

            geometry_msgs::Pose primitivePose;
            primitivePose.position.x = primitivePoses[i][0].as<double>();
            primitivePose.position.y = primitivePoses[i][1].as<double>();
            primitivePose.position.z = primitivePoses[i][2].as<double>();
            primitivePose.orientation.x = primitivePoses[i][3].as<double>();
            primitivePose.orientation.y = primitivePoses[i][4].as<double>();
            primitivePose.orientation.z = primitivePoses[i][5].as<double>();
            primitivePose.orientation.w = primitivePoses[i][6].as<double>();
            // Set the pose and primitive for the CollisionObject
            collisionObject.primitives.push_back(primitive);
            collisionObject.primitive_poses.push_back(primitivePose);
        }

        // Subframes
        for (std::size_t i = 0; i < subframeNames.size(); ++i)
        {
            geometry_msgs::Pose subframePose;
            subframePose.position.x = subframePoses[i][0].as<double>();
            subframePose.position.y = subframePoses[i][1].as<double>();
            subframePose.position.z = subframePoses[i][2].as<double>();
            subframePose.orientation.x = subframePoses[i][3].as<double>();
            subframePose.orientation.y = subframePoses[i][4].as<double>();
            subframePose.orientation.z = subframePoses[i][5].as<double>();
            subframePose.orientation.w = subframePoses[i][6].as<double>();
            // Set the pose and primitive for the CollisionObject
            collisionObject.subframe_names.push_back(subframeNames[i]);
            collisionObject.subframe_poses.push_back(subframePose);
        }

        // Mesh Poses
        for (std::size_t i = 0; i < meshPoses.size(); ++i)
        {
            geometry_msgs::Pose meshPose;
            meshPose.position.x = meshPoses[i][0].as<double>();
            meshPose.position.y = meshPoses[i][1].as<double>();
            meshPose.position.z = meshPoses[i][2].as<double>();
            meshPose.orientation.x = meshPoses[i][3].as<double>();
            meshPose.orientation.y = meshPoses[i][4].as<double>();
            meshPose.orientation.z = meshPoses[i][5].as<double>();
            meshPose.orientation.w = meshPoses[i][6].as<double>();
            // Set the pose and primitive for the CollisionObject
            collisionObject.subframe_poses.push_back(meshPose);
        }

        collisionObject.operation = moveit_msgs::CollisionObject::ADD;

        // Add the CollisionObject to the list
        scene_object_properties.collision_objects.push_back(collisionObject);
    }
}
