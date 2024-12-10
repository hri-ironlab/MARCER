using UnityEngine;
using TMPro;
using ARInterface.Core.Variables;
using ARInterface.Core; 
using ARInterface.Core.Events;
using MixedReality.Toolkit.UX;

namespace ARInterface.HUD
{
    public class RuleButtonDisplay : MonoBehaviour, IRuleDisplayStateResponse, IRuleMonitorRunningStateResponse
    {
        [SerializeField] private StringEvent _loggingEvent; 
        [SerializeField] private ActiveRulesVariable _activeRules;

        [SerializeField] private RuleDisplayState _currentRuleDisplayState;
        [SerializeField] private RuleMonitorRunningState _currentRuleMonitorRunningState; 

        // Client
        [SerializeField] private StringEvent _deleteRuleEvent;
        [SerializeField] private StringEvent _getRuleDescriptionEvent;
        [SerializeField] private StringEvent _pauseRuleEvent;

        // Server
        [SerializeField] private RuleMonitorRunningStateEvent _ruleMonitorRunningStateEvent; 
        [SerializeField] private RuleDisplayStateEvent _ruleDisplayStateEvent;

        [SerializeField] private int _ruleIndex = 0;

        private TextMeshProUGUI _text;
        private GameObject _highlight; 
        private FontIconSelector _iconSelector;

        private const string DELETE_ICON = "Icon 76";
        private const string PAUSE_ICON = "Icon 13";
        private const string THUMBS_UP_ICON = "Icon 138";
        private const string STOP_ICON = "Icon 126";
        private const string RUNNING_ICON = "Icon 93";
        private const string INFO_ICON = "Icon 127";

        bool logging_sent = false; 
        /// <summary>
        /// A Unity event function that is called on the frame when a script is enabled just before any of the update methods are called the first time.
        /// </summary> 
        private void Awake()
        {
            _iconSelector = gameObject.GetComponentInChildren<FontIconSelector>();
            _text = FindChildTextByName("Text");
            _highlight = gameObject.transform.GetChild(4).gameObject;
        }

        private void Update()
        {
            if (_activeRules.ActiveRules.rules.Length == 0) return;
            
            _text.text = _activeRules.ActiveRules.rules[_ruleIndex].name;

            bool isRuleActive = _activeRules.ActiveRules.rules[_ruleIndex].is_active;
            bool isExecutingRule = _activeRules.ActiveRules.executing_rule_index == _ruleIndex;
            
            if(_currentRuleDisplayState == RuleDisplayState.VIEW)
            {
                if(_currentRuleMonitorRunningState == RuleMonitorRunningState.RUNNING)
                {
                    
                    if(isExecutingRule){
                        _highlight.SetActive(isRuleActive);
                        if (!logging_sent)
                        {
                            _loggingEvent.Raise("Running Rule: " + _text.text);
                            logging_sent = true; 
                        }
                        SetIcon(RUNNING_ICON);
                    }
                    else
                    {
                        _highlight.SetActive(false);
                        if (logging_sent)
                        {
                            logging_sent = false; 
                        }
                        SetIcon(THUMBS_UP_ICON);
                    }
                }
                else if(_currentRuleMonitorRunningState == RuleMonitorRunningState.PAUSED)
                {
                    SetIcon(PAUSE_ICON);
                    _highlight.SetActive(isRuleActive);
                }
                else
                {
                    SetIcon(STOP_ICON);
                    _highlight.SetActive(false);
                }
            }
            else if (_currentRuleDisplayState == RuleDisplayState.DELETE)
            {
                SetIcon(DELETE_ICON);
                if(isExecutingRule){
                    _highlight.SetActive(isRuleActive);
                }
                else
                {
                    _highlight.SetActive(false);
                }
            }
            else if(_currentRuleDisplayState == RuleDisplayState.INFO)
            {
                SetIcon(INFO_ICON);
                if(isExecutingRule){
                    _highlight.SetActive(isRuleActive);
                }
                else
                {
                    _highlight.SetActive(false);
                }
            }
        }

        public void OnButtonPress(){
            if(_currentRuleDisplayState == RuleDisplayState.DELETE)
            {
                _loggingEvent.Raise("Deleting Rule: " + _text.text);
                _deleteRuleEvent.Raise(_text.text);
                _getRuleDescriptionEvent.Raise("");
            }
            else if(_currentRuleDisplayState == RuleDisplayState.INFO)
            {
                _loggingEvent.Raise("Getting and Displaying Rule Description: " + _text.text);
                _getRuleDescriptionEvent.Raise(_text.text);
            }
            else
            {
                if(_currentRuleMonitorRunningState == RuleMonitorRunningState.RUNNING)
                {
                    _loggingEvent.Raise("Pause/Run: " + _text.text);
                    _pauseRuleEvent.Raise(_text.text);
                }    
            }
        }

        public void OnRuleDisplayStateResponse(RuleDisplayState response)
        {
            if (_currentRuleDisplayState == response)
            {
                _currentRuleDisplayState = RuleDisplayState.VIEW; 
            }
            else{
                _currentRuleDisplayState = response;
            }

        }

        public void OnRuleMonitorRunningStateResponse(RuleMonitorRunningState response)
        {
            // If in stopped state, then only go to running state if Stopped pressed again
            if(_currentRuleMonitorRunningState == RuleMonitorRunningState.STOPPED)
            {
                if(response == RuleMonitorRunningState.STOPPED)
                {
                    _currentRuleMonitorRunningState = RuleMonitorRunningState.RUNNING;
                }
            }
            else
            {
                // If in RUNNING or PAUSED state, can switch to any state. 
                _currentRuleMonitorRunningState = response;
            }
        }

        private TextMeshProUGUI FindChildTextByName(string name)
        {
            foreach (var text in GetComponentsInChildren<TextMeshProUGUI>())
            {
                if (text.gameObject.name == name)
                {
                    return text;
                }
            }
            return null;
        }

        private void SetIcon(string iconName)
        {
            _iconSelector.CurrentIconName = iconName;
        }

        public void SetRuleIndex(int index)
        {
            _ruleIndex = index;
        }

        private void OnEnable()
        {
            _currentRuleDisplayState = RuleDisplayState.VIEW; 
            _ruleMonitorRunningStateEvent.RegisterListener(this);
            _ruleDisplayStateEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _ruleMonitorRunningStateEvent.UnregisterListener(this);
            _ruleDisplayStateEvent.UnregisterListener(this);
        }

    }
}
