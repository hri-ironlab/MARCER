using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.Std;

namespace ARInterface.ROS
{
    public class ListeningProgressSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/user_input/listening_progress";
        [SerializeField] private FloatVariable _listeningProgressVariable;

        void Start()
        {
            _ROS.Subscribe<Float32Msg>(_topicName, HandleMessage);
        }

        void HandleMessage(Float32Msg msg)
        {
            _listeningProgressVariable.Value = msg.data;
        }

        private void OnApplicationQuit()
        {
            _listeningProgressVariable.Clear();
        }
    }
}