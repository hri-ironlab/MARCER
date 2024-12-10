using UnityEngine;
using ARInterface.Core.Events;
using RosMessageTypes.Std;

namespace ARInterface.ROS
{
    public class DeleteTriggerPublisher : ROSBase, IStringResponse
    {
        [SerializeField] private string _pubTopicName = "/unity/delete_trigger";
        [SerializeField] private StringEvent _deleteTrigger;

        private void Start()
        {
            _ROS.RegisterPublisher<StringMsg>(_pubTopicName);
        }
        public void OnStringResponse(string response)
        {
            StringMsg msg = new StringMsg();
            msg.data = response;
            _ROS.Publish(_pubTopicName, msg);
        }

        public void OnEnable()
        {
            _deleteTrigger.RegisterListener(this);
        }

        public void OnDisable()
        {
            _deleteTrigger.UnregisterListener(this);
        }
    }
}