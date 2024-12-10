#include <world_monitor/world_monitor.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "world_monitor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    WorldMonitor world_monitor;
    ros::topic::waitForMessage<vision_msgs::Detection3DArray>("/perception/object_detections", nh, ros::Duration(10));

    world_monitor.object_detections_sub = nh.subscribe("/perception/object_detections", 10, &WorldMonitor::objectDetectionsCallback, &world_monitor);
    world_monitor.vision_info_sub = nh.subscribe("/perception/vision_info", 10, &WorldMonitor::visionInfoCallback, &world_monitor);
    world_monitor.planning_scene_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    world_monitor.update_planning_scene_service = nh.advertiseService("/world_monitor/update_planning_scene", &WorldMonitor::updatePlanningSceneRequest, &world_monitor);
    world_monitor.get_scene_objects_service = nh.advertiseService("/world_monitor/get_scene_objects", &WorldMonitor::getSceneObjectsRequest, &world_monitor);
    world_monitor.get_attached_object_service = nh.advertiseService("/world_monitor/get_attached_object", &WorldMonitor::getAttachedObjectRequest, &world_monitor);

    bool initialize_scene = true;
    std::string objects_file_path;
    if (!pnh.getParam("object_definitions_file_path", objects_file_path))
    {
        ROS_ERROR("Failed to retrieve object_definitions_file_path parameter.");
        return 1;
    }

    world_monitor.loadObjectParameters(objects_file_path);


    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        if (initialize_scene)
        {
            initialize_scene = world_monitor.initializePlanningScene();
        }
        else
        {
            world_monitor.broadcastTransforms();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
