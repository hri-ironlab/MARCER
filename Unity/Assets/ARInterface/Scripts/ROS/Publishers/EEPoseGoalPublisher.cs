using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.RelaxedIkRos1;
using ARInterface.Core;
using RosMessageTypes.Geometry;
using ARInterface.Core.Variables;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace ARInterface.ROS
{
    // Run this script to publish goals to relaxed IK algorithm
    public class EEPoseGoalPublisher : ROSBase
    {
        [SerializeField] private string _pubTopicName = "/relaxed_ik/ee_pose_goals";

        // Publishing rate information 
        [SerializeField] private double _publishRateHz = 100f;

        // Some robots have one end effector like the Fetch, others have two like the Baxter
        [SerializeField] private List<PoseVariable> _poseVariables;

        private double _lastPublishTimeSeconds;
        private double PublishPeriodSeconds => 1.0f / _publishRateHz;

        private bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > _lastPublishTimeSeconds;

        // List of messages with EE goals for each EE
        private PoseMsg[] _poseMsgs;


        private void Start()
        {
            _ROS.RegisterPublisher<EEPoseGoalsMsg>(_pubTopicName);

            // Add PoseMsg for each EELink
            _poseMsgs = new PoseMsg[_poseVariables.Count];
        }

        private void Update()
        {
            if (ShouldPublishMessage)
            {
                PublishMessage();
            }
        }

        private void PublishMessage()
        {
            var publishTime = Clock.time;

            for (int i = 0; i < _poseVariables.Count; i++)
            {
                var endEffectorPosition = _poseVariables[i].Position.To<FLU>();
                var endEffectorOrienation = _poseVariables[i].Rotation.To<FLU>();

                _poseMsgs[i] = new PoseMsg
                {
                    // Transform from Unity coordinates to ROS coordinates
                    position = new PointMsg(endEffectorPosition.x, endEffectorPosition.y, endEffectorPosition.z),
                    orientation = new QuaternionMsg(endEffectorOrienation.x, endEffectorOrienation.y, endEffectorOrienation.z, endEffectorOrienation.w),
                };
            }

            // Add header and pose msgs to new end effector pos goal message
            EEPoseGoalsMsg eePoseGoalMsg = new EEPoseGoalsMsg
            {
                header = new HeaderMsg(),
                ee_poses = _poseMsgs
            };

            _ROS.Publish(_pubTopicName, eePoseGoalMsg);
            _lastPublishTimeSeconds = publishTime;
        }
    }
}