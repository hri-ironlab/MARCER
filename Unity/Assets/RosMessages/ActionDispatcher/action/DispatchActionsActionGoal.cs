using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.ActionDispatcher
{
    public class DispatchActionsActionGoal : ActionGoal<DispatchActionsGoal>
    {
        public const string k_RosMessageName = "action_dispatcher/DispatchActionsActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public DispatchActionsActionGoal() : base()
        {
            this.goal = new DispatchActionsGoal();
        }

        public DispatchActionsActionGoal(HeaderMsg header, GoalIDMsg goal_id, DispatchActionsGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static DispatchActionsActionGoal Deserialize(MessageDeserializer deserializer) => new DispatchActionsActionGoal(deserializer);

        DispatchActionsActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = DispatchActionsGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
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
