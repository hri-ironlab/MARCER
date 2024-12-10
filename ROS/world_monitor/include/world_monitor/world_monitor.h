#ifndef WORLD_MONITOR_H
#define WORLD_MONITOR_H

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/VisionInfo.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <world_monitor/GetAttachedObject.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/TransformStamped.h>
#include <shape_msgs/Mesh.h>
#include <yaml-cpp/yaml.h>

class WorldMonitor
{
public:
    WorldMonitor();
    ~WorldMonitor() = default;

    // PSI
    bool initializePlanningScene();
    bool updatePlanningSceneRequest(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool getSceneObjectsRequest(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res);
    bool applyPlanningScene(moveit_msgs::PlanningScene &planning_scene);
    void loadObjectParameters(const std::string objects_filepath);
    bool getAttachedObjectRequest(world_monitor::GetAttachedObject::Request &req, world_monitor::GetAttachedObject::Response &res);
    std::string GetAttachedObject();
    
    moveit_msgs::PlanningScene updatePlanningScene();
    moveit_msgs::CollisionObject getObjectMesh(const std::string &name, moveit_msgs::CollisionObject &collisionObject);

    // Object detection/tracking
    void objectDetectionsCallback(const vision_msgs::Detection3DArray::ConstPtr &msg);
    void visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr &msg);
    void addObjectTransform(const moveit_msgs::CollisionObject &collisionObject);
    void broadcastTransforms();

    // ROS Communication
    ros::Subscriber object_detections_sub;
    ros::Subscriber vision_info_sub;
    ros::ServiceClient planning_scene_client;
    ros::ServiceServer update_planning_scene_service;
    ros::ServiceServer get_scene_objects_service;
    ros::ServiceServer get_attached_object_service;
    tf2_ros::TransformBroadcaster tfb;

private:
    vision_msgs::Detection3DArray object_detections;
    moveit_msgs::PlanningSceneWorld active_scene_objects;
    moveit_msgs::PlanningSceneWorld scene_object_properties;
    moveit_msgs::PlanningSceneWorld surface_object_properties;
    std::vector<std::string> objects_info;
    std::vector<geometry_msgs::TransformStamped> transformStampedArray;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};

#endif