using UnityEngine;
using RosMessageTypes.Gazebo;
using ARInterface.Core.Variables;
using UnityEngine.Assertions.Must;

namespace ARInterface.ROS
{
    public class ModelStatesSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/unity/model_states";
        [SerializeField] private ModelStatesVariable _modelStatesVariable;

        void Start()
        {
            _ROS.Subscribe<ModelStatesMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(ModelStatesMsg msg)
        {
            _modelStatesVariable.ModelStates = msg;
        }

        private void OnApplicationQuit()
        {
            _modelStatesVariable.ModelStates.name = null;
            _modelStatesVariable.ModelStates.pose = null;
        }
    }
}