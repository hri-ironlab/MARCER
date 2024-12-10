using System.Collections.Generic;
using System.Linq;
using Codice.Client.BaseCommands;
//using RosMessageTypes.TriggerActionManager;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace ARInterface.TAP
{
    public class TriggerActionVisual : MonoBehaviour
    {
        /*[SerializeField]
        private List<GameObject> buttons;
        [SerializeField]
        private List<TextMeshPro> texts;
        [SerializeField]
        private List<GameObject> toggleQuads;

        private const string TopicName = "/unity/update_trigger_list";
        private const string ServiceName = "/trigger_action_node/apply_trigger";
        private const string PreString = "Rule ";

        private string triggerName;
        private ROSConnection rosConnection;
        [SerializeField] private UpdateTriggerListMsg updateTriggerListMsg;

        private void Awake()
        {
            InitializeROSConnection();
        }

        private void Update()
        {
            DisplayTriggers();
        }

        private void InitializeROSConnection()
        {
            rosConnection = ROSConnection.GetOrCreateInstance();
            rosConnection.Subscribe<UpdateTriggerListMsg>(TopicName, HandleUpdateTriggerListMessage);
            rosConnection.RegisterRosService<ApplyTriggerRequest, ApplyTriggerResponse>(ServiceName);
        }

        private void HandleUpdateTriggerListMessage(UpdateTriggerListMsg message)
        {
            updateTriggerListMsg = message; 
        }

        private void DisplayTriggers()
        {
            int tapCount = updateTriggerListMsg.triggers.Length;

            for (int i = 0; i < buttons.Count; i++)
            {
                
                if (i < tapCount)
                {
                    var trigger = updateTriggerListMsg.triggers[i];
                    buttons[i].SetActive(true);
                    if(trigger.trigger_type == TriggerMsg.OBJECT_ZONE)
                    {
                        triggerName = "IF objects are inside " + updateTriggerListMsg.triggers[i].zone_name;
                    }
                    else
                    {
                        triggerName = "IF " + updateTriggerListMsg.triggers[i].wait_time.ToString() + " seconds have passed";
                    }

                    
                    if(updateTriggerListMsg.is_executing_id == trigger.id)
                    {
                        toggleQuads[i].SetActive(true);
                        texts[i].text = $"{PreString}{i + 1} EXECUTING\nTrigger->{triggerName}\nAction ->{trigger.action}";
                    }
                    else if (updateTriggerListMsg.active_trigger_ids.Contains(trigger.id))
                    {
                        toggleQuads[i].SetActive(false);
                        texts[i].text = $"{PreString}{i + 1} ACTIVE\nTrigger->{triggerName}\nAction ->{trigger.action}";
                    }
                    else
                    {
                        toggleQuads[i].SetActive(false);
                        texts[i].text = $"{PreString}{i + 1}\nTrigger->{triggerName}\nAction ->{trigger.action}";
                    }
                }
                else
                {
                    buttons[i].SetActive(false);
                    toggleQuads[i].SetActive(false);
                }
            }
        }

        public void DeleteRule(int ruleIndex)
        {
            var request = new ApplyTriggerRequest
            {
                trigger = { id = updateTriggerListMsg.triggers[ruleIndex].id, operation = TriggerMsg.DELETE }
            };

            rosConnection.SendServiceMessage<ApplyTriggerResponse>(ServiceName, request, HandleDeleteResponse);
        }

        private void HandleDeleteResponse(ApplyTriggerResponse response)
        {
            // Optional: Implement logic to handle the response from deleting a trigger.
        }*/
    }
}
