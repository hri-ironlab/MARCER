using UnityEngine;
using RosMessageTypes.ActionDispatcher;
using ARInterface.Core.Variables;

namespace ARInterface.ROS
{
    public class GetSolutionServer : ROSBase
    {
        [SerializeField] private string _serverTopicName = "action_dispatcher/send_solution";
        [SerializeField] private DisplaySolutionVariable _displaySolutionVariable;


        private void Start()
        {
            _ROS.ImplementService<SendSolutionRequest, SendSolutionResponse>(_serverTopicName, UpdateSolution);
        }

        private SendSolutionResponse UpdateSolution(SendSolutionRequest req)
        {
            _displaySolutionVariable.Solution = req.solution;
            return new SendSolutionResponse(true);
        }
    }
}