using ARInterface.Core.Variables;
using UnityEngine;
using TMPro;
using RosMessageTypes.Manipulation;
using System.Collections;
using RosMessageTypes.RuleGenerator;
using RosMessageTypes.ActionDispatcher;
using ARInterface.Core.Events;
using System.Collections.Generic;
using ARInterface.TAP;
using ARInterface.Core;
using UnityEngine.UI;
using System;
using log4net.Core;

namespace ARInterface.HUD
{
    public class HUD : MonoBehaviour, IStringResponse
    {
        [SerializeField] private FloatVariable _listeningProgressVariable;
        [SerializeField] private RuleGeneratorFeedbackVariable _ruleGeneratorFeedbackVariable;
        [SerializeField] private TextMeshPro _headerText;
        [SerializeField] private TextMeshPro _bodyText;
        [SerializeField] private UserApprovalResponseEvent _getUserApprovalResponse;
        [SerializeField] private StringEvent _getUserApprovalRequest;
        [SerializeField] private GameObject _dialogButtons;
        [SerializeField] private GameObject _mainMenuButtons;
        [SerializeField] private RuleGeneratorStateVariable _state;
        [SerializeField] private ActiveTriggerHolder _activeTriggerHolder;
        private GameObject activeTriggerHolder;
        private string _computedPlan = ""; 
        private int maxDots = 3;
        private int _numberOfDashes = 52;
        [SerializeField] private bool delay = false;
        bool showRelevantObjects = true; 
        [SerializeField] private List<GameObject> _gameObjectModels;
        [SerializeField] private WorldObjectDictionary _worldObjectDictionary;

        [SerializeField] private StringEvent _loggingEvent;
        private RuleGeneratorStateMsg _ruleGeneratorStateMsg = new RuleGeneratorStateMsg();
        bool isgivingfeedback = false;
        public string highlightPlan = ""; 
        List<GameObject> _highlightedObjects = new List<GameObject>();
        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            if (!delay)
            {
                switch(_state.State.state)
                {
                    case RuleGeneratorStateMsg.IDLE:
                        ShowListeningProgress();
                        break;
                    case RuleGeneratorStateMsg.GENERATE_RULE:
                        isgivingfeedback = false;
                        ShowRelevantObjects(false);
                        highlightPlan = "";
                        ShowRuleGeneratorFeedback();
                        ShowCreatingAPlan();
                        break;
                    case RuleGeneratorStateMsg.WAIT_FOR_APPROVAL:
                        showRelevantObjects = true;
                        ShowDialog();
                        break;
                    case RuleGeneratorStateMsg.WAIT_FOR_FEEDBACK:
                        isgivingfeedback = true;
                        ShowFeedbackDisplay();
                        ShowDialog();
                        break;
                    case RuleGeneratorStateMsg.ADD_RULE:
                        isgivingfeedback = false;
                        ShowRelevantObjects(false);
                        ClearHeader();
                        DisplayRuleAdded();
                        break;
                    case RuleGeneratorStateMsg.DONE:
                        ClearBody();
                        ShowRelevantObjects(false);
                        ShowListeningProgress();
                        break;
                }
                if (_state.State.state != RuleGeneratorStateMsg.WAIT_FOR_APPROVAL)
                {
                    _mainMenuButtons.SetActive(true);
                    _dialogButtons.SetActive(false);
                }
            }
        }

        private void ClearHeader()
        {
            _headerText.text = "";
        }
        private void ClearBody()
        {
            _bodyText.text = "";
        }
         
        private void ShowCreatingAPlan()
        {
            _bodyText.text = "Building a plan for this command...\n";
        }
        private void ShowDialog()
        {
            string intro;
            if (isgivingfeedback)
            {
                intro = "Provide feedback for the following plan...\n";
            }
            else
            {
                intro = "Choose how to proceed with the following plan...\n";
            }
            _mainMenuButtons.SetActive(false);
            _dialogButtons.SetActive(true);
            _bodyText.text = intro + _computedPlan;
            SendLogging(_bodyText.text);
            if (showRelevantObjects)
            {
                ShowRelevantObjects(showRelevantObjects);
            }
        }


        public void ShowRelevantObjects(bool isActive)
        {
            foreach(GameObject obj in _gameObjectModels)
            {
                if (highlightPlan.Contains(obj.name))
                {
                    var pose = _worldObjectDictionary.Items[obj.name];
                    obj.SetActive(isActive);
                    obj.transform.SetPositionAndRotation(pose.transform.position, pose.transform.rotation);
                }
            }

            foreach(GameObject obj in _activeTriggerHolder._activeTriggers)
            {
                if (highlightPlan.Contains(obj.name))
                {
                    obj.GetComponent<Outline>().enabled = isActive;
                }
            }
        }

        static string TrimAndCapitalize(string input)
        {
            // Trim leading and trailing spaces, and split into words
            string[] words = input.Trim().Split(' ');

            if (!string.IsNullOrEmpty(words[0]))
            {
                // Capitalize the first letter of the word
                words[0] = char.ToUpper(words[0][0]) + words[0].Substring(1);
            }

            // Join the words back into a single string
            return string.Join(" ", words);
        }

        private void DisplayRuleAdded()
        {
            _bodyText.text = "Rule added. Click or say \"Show Rules\" to see your rules.";
            delay = true;
            SendLogging(_bodyText.text);

            StartCoroutine(DisplayDelay());
        }
        private void ShowRuleGeneratorFeedback()
        {
            if(_ruleGeneratorFeedbackVariable.Feedback != null)
            {
                if(_ruleGeneratorFeedbackVariable.Feedback.speech_command != "")
                {
                    _headerText.text = "I heard: \"" + TrimAndCapitalize(_ruleGeneratorFeedbackVariable.Feedback.speech_command) + ". ";

                    if (_ruleGeneratorFeedbackVariable.Feedback.speech_command_feedback != "")
                    {
                        _headerText.text += TrimAndCapitalize(_ruleGeneratorFeedbackVariable.Feedback.speech_command_feedback); 
                    }
                    SendLogging(_headerText.text);
                    _headerText.text += "\"\n--------------------------------------------------------\n";
                    // for (int i = 0; i <= maxDots; i++)
                    // {
                    //     if (i != 0)
                    //     {
                    //         _headerText.text += ".";
                    //     }
                    //     yield return new WaitForSeconds(1);
                    // }
                }
            }
        }

        private void SendLogging(string text)
        {
            if(_state.State.state != _ruleGeneratorStateMsg.state)
            {
                _loggingEvent.Raise(text); 
                _ruleGeneratorStateMsg.state = _state.State.state;
            }
        }

        private IEnumerator DisplayDelay()
        {
            yield return new WaitForSeconds(2);
            delay = false; 
        }

        private void ShowFeedbackDisplay()
        {
            float progressValue = Mathf.Clamp01(_listeningProgressVariable.Value); // Clamping the progress value between 0 and 1
            int filledDashes = Mathf.RoundToInt(progressValue * _numberOfDashes); // Calculating filled dashes

            string progressString = "Say 'Hey Fetch [Feedback to fix plan...]'\n";
            progressString += "[";

            // Adding filled dashes
            for (int i = 0; i < filledDashes; i++)
            {
                progressString += "-";
            }

            // Adding empty dashes
            for (int i = filledDashes; i < _numberOfDashes; i++)
            {
                progressString += "_";
            }

            progressString += "]";

            // Displaying the progress text
            _headerText.text = progressString + "\n\n";
        }

        private void ShowListeningProgress()
        {
            float progressValue = Mathf.Clamp01(_listeningProgressVariable.Value); // Clamping the progress value between 0 and 1
            int filledDashes = Mathf.RoundToInt(progressValue * _numberOfDashes); // Calculating filled dashes

            string progressString = "Say 'Hey Fetch [Command]'\n";
            progressString += "[";

            // Adding filled dashes
            for (int i = 0; i < filledDashes; i++)
            {
                progressString += "-";
            }

            // Adding empty dashes
            for (int i = filledDashes; i < _numberOfDashes; i++)
            {
                progressString += "_";
            }

            progressString += "]";
             
            // Displaying the progress text
            _headerText.text = progressString;
        }

        public void OnStringResponse(string response)
        {
            string keyword = "split";
            int index = response.IndexOf(keyword); 
            string before = response.Substring(0, index).Trim();
            string after = response.Substring(index + keyword.Length).Trim();
            _computedPlan = before;
            highlightPlan = after; 
        }

        public void Approve()
        {
            ShowRelevantObjects(false);
            showRelevantObjects = false;
            _mainMenuButtons.SetActive(true);
            _dialogButtons.SetActive(false);
            _getUserApprovalResponse.Raise(new UserApprovalResponseMsg(UserApprovalResponseMsg.APPROVED));
        }
        public void GiveFeedback()
        {
            ShowRelevantObjects(false);
            _ruleGeneratorFeedbackVariable.Clear();
            showRelevantObjects = false;
            _mainMenuButtons.SetActive(true);
            _dialogButtons.SetActive(false);
            _getUserApprovalResponse.Raise(new UserApprovalResponseMsg(UserApprovalResponseMsg.FEEDBACK));
        }
        public void Delete()
        {
            ShowRelevantObjects(false);
            _computedPlan = "";
            _ruleGeneratorFeedbackVariable.Clear();
            showRelevantObjects = false;
            _mainMenuButtons.SetActive(true);
            _dialogButtons.SetActive(false);
            _getUserApprovalResponse.Raise(new UserApprovalResponseMsg(UserApprovalResponseMsg.DELETE));
        }

        private void OnEnable()
        {
            _getUserApprovalRequest.RegisterListener(this);
        }

        private void OnDisable()
        {
            _getUserApprovalRequest.UnregisterListener(this);
        }

        private void OnApplicationQuit()
        {
            _listeningProgressVariable.Clear();
            _ruleGeneratorFeedbackVariable.Clear();
            _state.Clear();
        }
    }

}
