using UnityEngine;
using RosMessageTypes.RuleGenerator; 

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/UserApprovalResponseEvent")]
    public class UserApprovalResponseEvent : EventBase<IUserApprovalResponse>
    {
        public void Raise(UserApprovalResponseMsg response)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnUserApprovalResponse(response);
        }
    }
}