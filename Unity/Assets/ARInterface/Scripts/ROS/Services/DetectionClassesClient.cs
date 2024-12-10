using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.Moveit;
using RosMessageTypes.Perception;

namespace ARInterface.ROS
{
    public class DetectionClassesClient : ROSBase
    {
        [SerializeField] private string _serviceName = "/perception/get_detection_classes";
        [SerializeField] private StringArrayVariable _detectionClasses;

        private void Start()
        {
            _ROS.RegisterRosService<GetDetectionClassesRequest, GetDetectionClassesResponse>(_serviceName);
            _ROS.SendServiceMessage<GetDetectionClassesResponse>(_serviceName, new GetDetectionClassesRequest(), HandleGetDetectionClassesResponse);
        }

        private void HandleGetDetectionClassesResponse(GetDetectionClassesResponse response)
        {
            if (response.detection_classes.Length > 0)
            {
                _detectionClasses.Values = response.detection_classes;
                Log("DetectionClassesService", "Detection classes received.");
            }
            else
            {
                Log("DetectionClassesService", "No detection classes returned, is the perception ROS package running?");
            }
        }
    }
}