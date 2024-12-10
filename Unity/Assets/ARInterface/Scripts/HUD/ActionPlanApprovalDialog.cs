using ARInterface.Core; 
using UnityEngine;
using MixedReality.Toolkit.UX;
using ARInterface.Core.Events;
using RosMessageTypes.RuleGenerator;

namespace ARInterface.HUD
{
    public class ActionPlanApprovalDialog : MonoBehaviour, IStringResponse
    {
        [SerializeField] private DialogPool _dialogPool;
        [SerializeField] private UserApprovalResponseEvent _getUserApprovalResponse;
        [SerializeField] private StringEvent _getActionPlanApprovalRequest; 
        // Start is called before the first frame update
        void Start()
        {
            _dialogPool = GetComponent<DialogPool>();
        }

        public void OnStringResponse(string response)
        {
            IDialog dialog = _dialogPool.Get()
                .SetHeader("Rule Generator Approval")
                .SetBody(response)
                .SetPositive("Approve", (args) => _getUserApprovalResponse.Raise(new UserApprovalResponseMsg(UserApprovalResponseMsg.APPROVED)))
                .SetNegative("Give Feedback", (args) => { _getUserApprovalResponse.Raise(new UserApprovalResponseMsg(UserApprovalResponseMsg.FEEDBACK)); ShowFeedbackDialogue(response); })
                .SetNeutral("Delete", (args) => _getUserApprovalResponse.Raise(new UserApprovalResponseMsg(UserApprovalResponseMsg.DELETE)));
            dialog.Show();
        }

        public void ShowFeedbackDialogue(string body)
        {
            IDialog dialog = _dialogPool.Get()
                .SetHeader("Provide feedback by saying, 'Hey Fetch, [How to fix the plan]...'")
                .SetBody(body);
            dialog.Show();
        }

        private void OnEnable()
        {
            _getActionPlanApprovalRequest.RegisterListener(this);
        }

        private void OnDisable()
        {
            _getActionPlanApprovalRequest.UnregisterListener(this);
        }
    }
}
