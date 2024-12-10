using UnityEngine;
using RosMessageTypes.SceneGraph;
using ARInterface.TAP;
using System.Collections.Generic;
using ARInterface.Core;
using RosMessageTypes.Geometry;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace ARInterface.ROS
{
    public class UnityRelationshipsPublisher : ROSBase
    {
        [SerializeField] private string _pubTopicName = "/scene_graph/unity_relationships";
        [SerializeField] private double _publishRateHz = 100f;
        [SerializeField] private GameObjectSet _activeTriggers;


        private double _lastPublishTimeSeconds;
        private double PublishPeriodSeconds => 1.0f / _publishRateHz;
        private bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > _lastPublishTimeSeconds;

        // Start is called before the first frame update
        void Start()
        {
            // Get ROS connection static instance
            _ROS.RegisterPublisher<UnityRelationshipsMsg>(_pubTopicName);
        }

        private void Update()
        {
            if (ShouldPublishMessage)
            {
                PublishMessage();
            }
        }

        public void PublishMessage()
        {
            var publishTime = Clock.time;

            UnityRelationshipMsg[] relationships = new UnityRelationshipMsg[_activeTriggers.Items.Count];
             
            for (int i = 0; i < _activeTriggers.Items.Count; i++)
            {
                var trigger = _activeTriggers.Items[i];
                var scale = trigger.transform.localScale.To<FLU>();
                scale.y = -scale.y;
                double[] dimensions = new double[] { scale.x, scale.y, scale.z };
                relationships[i] = new UnityRelationshipMsg
                {
                    zone = trigger.gameObject.name,
                    relationship = "has_inside",
                    node_objects = trigger.GetComponent<ZoneTrigger>().GetObjectsInside().ToArray(),
                    pose = new PoseMsg(trigger.transform.localPosition.To<FLU>(), trigger.transform.localRotation.To<FLU>()),
                    primitive = new SolidPrimitiveMsg(SolidPrimitiveMsg.BOX, dimensions)
                };
            }

            UnityRelationshipsMsg msg = new() { relationships = relationships };
            _ROS.Publish(_pubTopicName, msg);
            _lastPublishTimeSeconds = publishTime;
        }
    }
}