using UnityEngine;
using RosMessageTypes.RuleGenerator;
using ARInterface.Core.Events;
using Unity.Robotics.ROSTCPConnector;
using System.Threading.Tasks;

namespace ARInterface.ROS
{
    public class GetUserApprovalServer : ROSBase, IUserApprovalResponse
    {
        [SerializeField] private string _serverTopicName = "/get_user_approval";
        [SerializeField] private StringEvent _getActionPlanApprovalRequest;
        [SerializeField] private UserApprovalResponseEvent _getUserApprovalResponse;

        private GetUserApprovalResponse _userApprovalResponse = new(); 
        private bool _userDialogResponseReceived = false;

        private void Start()
        {
            _ROS.ImplementService<GetUserApprovalRequest, GetUserApprovalResponse>(_serverTopicName, GetUserApproval);
        }

        private async Task<GetUserApprovalResponse> GetUserApproval(GetUserApprovalRequest request)
        {
            _getActionPlanApprovalRequest.Raise(request.plan_description + " split " + request.generated_function);
            while (!_userDialogResponseReceived)
                await Task.Yield();
            _userDialogResponseReceived = false; 
            return _userApprovalResponse;
        }

        public void OnUserApprovalResponse(UserApprovalResponseMsg response)
        {
            _userApprovalResponse.response = response;
            _userDialogResponseReceived = true;
        }

        private void OnEnable()
        {
            _getUserApprovalResponse.RegisterListener(this);
        }

        private void OnDisable()
        {
            _getUserApprovalResponse.UnregisterListener(this);
        }
    }
}