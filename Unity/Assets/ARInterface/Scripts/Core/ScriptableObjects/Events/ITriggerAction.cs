namespace ARInterface.Core.Events
{
    public interface ITriggerAction
    {
        void OnTriggerActionEvent(int id, string action);
    }
}
