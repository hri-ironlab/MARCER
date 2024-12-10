using UnityEngine;
using RosMessageTypes.Moveit;
using RosMessageTypes.Std;

namespace ARInterface.ROS
{
    public class StopRobotClient : ROSBase
    {
        [SerializeField] private string _serviceName = "/executive/stop_robot";

        private void Start()
        {
            _ROS.RegisterRosService<TriggerRequest, TriggerResponse>(_serviceName);
        }
        public void RequestStopRobot()
        {
            _ROS.SendServiceMessage<TriggerResponse>(_serviceName, new TriggerRequest(), HandlePreemptExecutiveResponse);
        }
        private void HandlePreemptExecutiveResponse(TriggerResponse response)
        {
            if (response.success)
            {
                Log("StopRobotService", response.message);
            }
            else
            {
                Log("StopRobotService", response.message);
            }
        }
    }
}