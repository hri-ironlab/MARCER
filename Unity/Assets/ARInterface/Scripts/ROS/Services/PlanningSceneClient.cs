using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.Moveit; 

namespace ARInterface.ROS
{
    public class PlanningSceneClient : ROSBase
    {
        [SerializeField] private string _serviceName = "/world_monitor/get_scene_objects";
        [SerializeField] private PlanningSceneWorldVariable _planningSceneWorldVariable;

        private void Start()
        {
            _planningSceneWorldVariable.Clear();
            _ROS.RegisterRosService<GetPlanningSceneRequest, GetPlannerParamsResponse>(_serviceName);
            _ROS.SendServiceMessage<GetPlanningSceneResponse>(_serviceName, new GetPlanningSceneRequest(), HandleGetPlanningSceneResponse);
        }

        private void HandleGetPlanningSceneResponse(GetPlanningSceneResponse response)
        {
            if (response.scene.world.collision_objects.Length > 0)
            {
                _planningSceneWorldVariable.PlanningSceneWorld = response.scene.world;
                Log("PlanningSceneService", "Initializing planning scene.");
            }
            else
            {
                Log("PlanningSceneService", "No static scene returned, is the world_monitor ROS package running?");
            }
        }
    }
}