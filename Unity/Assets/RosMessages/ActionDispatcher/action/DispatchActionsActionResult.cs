using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ActionDispatcher
{
    public class DispatchActionsActionResult : ActionResult<DispatchActionsResult>
    {
        public const string k_RosMessageName = "action_dispatcher/DispatchActionsActionResult";
        public override string RosMessageName => k_RosMessageName;


        public DispatchActionsActionResult() : base()
        {
            this.result = new DispatchActionsResult();
        }

        public DispatchActionsActionResult(HeaderMsg header, GoalStatusMsg status, DispatchActionsResult result) : base(header, status)
        {
            this.result = result;
        }
        public static DispatchActionsActionResult Deserialize(MessageDeserializer deserializer) => new DispatchActionsActionResult(deserializer);

        DispatchActionsActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = DispatchActionsResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
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
