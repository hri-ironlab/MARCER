using UnityEngine;
using ARInterface.Core.Variables;

namespace ARInterface.Tools
{
    public class ConfigurationManager : MonoBehaviour
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private bool _useActiveTopicConfig;
        [SerializeField] private string _activeTopicNameValue; 
        [SerializeField] private StringVariable _activeTopicName;
        [SerializeField] private BoolVariable _isOtherLabelBeingDisplayed;

        private void Start()
        {
            SetActiveTopicName(); 
        }

        private void SetActiveTopicName()
        {
            if (_activeTopicName != null && _useActiveTopicConfig)
            {
                _activeTopicName.Value = _activeTopicNameValue; 
            }
        }

        private void OnDestroy()
        {
            _isOtherLabelBeingDisplayed.Clear(); 
        }
    }
}
