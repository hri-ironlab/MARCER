using UnityEngine;
using TMPro;
using ARInterface.Core.Variables;

namespace ARInterface.Core
{
    public class TimedLabelDisplay : MonoBehaviour
    {
        [SerializeField] private GameObject _label;
        [SerializeField] private TextMeshPro _labelText;
        [SerializeField] private float _resetTime = 1f;
        [SerializeField] private float _offset = .02f;
        [SerializeField] private BoolVariable _isOtherLabelBeingDisplayed; 
        private float _timeShown = float.MinValue;
        private Transform _targetTransform;
        private bool _isLooking = false;
        private bool _currentlyDisplayed = false; 

        private void Update()
        {
            transform.position = new Vector3(_targetTransform.position.x, _targetTransform.position.y + _offset, _targetTransform.position.z);

            if (_isLooking)
            {
                if (!_isOtherLabelBeingDisplayed.Value)
                {
                    _currentlyDisplayed = true; 
                    _label.gameObject.SetActive(true);
                    _isOtherLabelBeingDisplayed.Value = true;

                }
            }
            else
            {
                if (_currentlyDisplayed)
                {
                    bool _isDisplaying = Time.time - _timeShown <= _resetTime;
                    if (!_isDisplaying)
                    {
                        _label.gameObject.SetActive(false);
                        _isOtherLabelBeingDisplayed.Value = false;
                        _currentlyDisplayed = false; 
                    }
                }
            }
        }
        public void SetProperties(Transform targetTransform, string label)
        {
            _targetTransform = targetTransform;
            _offset += _targetTransform.localScale.y / 2f;
            _labelText.text = label;
            name = label;
            _label.gameObject.SetActive(false); 
        }
        public void DisplayLabel()
        {
            _isLooking = !_isLooking;
            _timeShown = Time.time;
        }

        private void OnDestroy()
        {
            if (_currentlyDisplayed)
            {
                _isOtherLabelBeingDisplayed.Value = false;
            }
        }
    }
}