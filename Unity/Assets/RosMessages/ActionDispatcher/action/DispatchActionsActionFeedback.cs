using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ActionDispatcher
{
    public class DispatchActionsActionFeedback : ActionFeedback<DispatchActionsFeedback>
    {
        public const string k_RosMessageName = "action_dispatcher/DispatchActionsActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public DispatchActionsActionFeedback() : base()
        {
            this.feedback = new DispatchActionsFeedback();
        }

        public DispatchActionsActionFeedback(HeaderMsg header, GoalStatusMsg status, DispatchActionsFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static DispatchActionsActionFeedback Deserialize(MessageDeserializer deserializer) => new DispatchActionsActionFeedback(deserializer);

        DispatchActionsActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = DispatchActionsFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
        }


#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
