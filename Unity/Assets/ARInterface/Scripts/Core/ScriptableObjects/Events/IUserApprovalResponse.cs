using RosMessageTypes.RuleGenerator;

namespace ARInterface.Core.Events
{
    public interface IUserApprovalResponse
    {
        void OnUserApprovalResponse(UserApprovalResponseMsg response);
    }
}
