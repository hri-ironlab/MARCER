using RosMessageTypes.Sensor;
using RosMessageTypes.LfdReceiver; 
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Std;
using ARInterface.Core;

namespace ARInterface.ROS
{
    public class DemonstrationPublisher : MonoBehaviour
    {

        [SerializeField] private string _subTopicName = "/relaxed_ik/joint_angle_solutions";
        [SerializeField] private string _rosServiceName = "/unity/record";
        [SerializeField] private string _pubTopicName = "/unity/demonstration";
        [SerializeField] private double _publishRateHz = 100f;

        private double _lastPublishTimeSeconds;
        private double PublishPeriodSeconds => 1.0f / _publishRateHz;
        private bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > _lastPublishTimeSeconds;

        private JointStateMsg _jointStates;
        // ROS Connector
        private ROSConnection _ROS;

        private bool _picking = false;
        private bool _placing = false;

        // Start is called before the first frame update
        void Start()
        {
            // Get ROS connection static instance
            _ROS = ROSConnection.GetOrCreateInstance();
            _ROS.RegisterRosService<RecordRequest, RecordResponse>(_rosServiceName);
            _ROS.Subscribe<JointStateMsg>(_subTopicName, HandleJointAngles);
            _ROS.RegisterPublisher<DemonstrationMsg>(_pubTopicName);
        }

        private void Update()
        {
            if (ShouldPublishMessage)
            {
                PublishMessage();
            }
        }
        public void SignalToRecord()
        {
            var request = new RecordRequest();
            request.begin.data = true;

            _ROS.SendServiceMessage<RecordResponse>(_rosServiceName, request, ReceivedResponse);
        }
        public void pick()
        {
            _picking = true;
            _placing = false;
        }

        public void place()
        {
            _placing = true;
            _picking = false;
        }

        private void HandleJointAngles(JointStateMsg msg)
        {
            _jointStates = msg;
        }
        
        public void PublishMessage()
        {
            var publishTime = Clock.time;

            DemonstrationMsg msg = new DemonstrationMsg
            {
                header = new HeaderMsg(),
                joint_states = _jointStates,
                pick = new BoolMsg(_picking),
                place = new BoolMsg(_placing)
            };

            _ROS.Publish(_pubTopicName, msg);
            _lastPublishTimeSeconds = publishTime;
            _placing = false;
        }
        private void ReceivedResponse(RecordResponse response)
        {
            Debug.Log("Received response: " + response.received.data);
        }
    }
}