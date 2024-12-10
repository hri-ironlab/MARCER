using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.ActionDispatcher
{
    public class DispatchActionsAction : Action<DispatchActionsActionGoal, DispatchActionsActionResult, DispatchActionsActionFeedback, DispatchActionsGoal, DispatchActionsResult, DispatchActionsFeedback>
    {
        public const string k_RosMessageName = "action_dispatcher/DispatchActionsAction";
        public override string RosMessageName => k_RosMessageName;


        public DispatchActionsAction() : base()
        {
            this.action_goal = new DispatchActionsActionGoal();
            this.action_result = new DispatchActionsActionResult();
            this.action_feedback = new DispatchActionsActionFeedback();
        }

        public static DispatchActionsAction Deserialize(MessageDeserializer deserializer) => new DispatchActionsAction(deserializer);

        DispatchActionsAction(MessageDeserializer deserializer)
        {
            this.action_goal = DispatchActionsActionGoal.Deserialize(deserializer);
            this.action_result = DispatchActionsActionResult.Deserialize(deserializer);
            this.action_feedback = DispatchActionsActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
