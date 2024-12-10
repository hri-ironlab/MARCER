using ARInterface.Core.Variables;
using UnityEngine;
using RosMessageTypes.Vision;

namespace ARInterface.ROS
{
    public class ObjectDetectionsSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/perception/object_detections";
        [SerializeField] private Detection3DArrayVariable _detections; 
        void Start()
        {
            _detections.Clear();
            _ROS.Subscribe<Detection3DArrayMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(Detection3DArrayMsg msg)
        {
            _detections.SetValue(msg);
        }
    }
}