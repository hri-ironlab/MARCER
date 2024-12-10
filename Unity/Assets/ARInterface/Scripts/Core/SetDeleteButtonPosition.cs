using UnityEngine;
using TMPro;
using ARInterface.Core.Events;

using MixedReality.Toolkit.UX; 
namespace ARInterface.Core
{
    public class DeleteButtonManager : MonoBehaviour
    {
        [SerializeField] private GameObject _button;
        [SerializeField] private TMP_Text _text;
        [SerializeField] private float _labelSeparation = 0.032f;
        [SerializeField] private StringEvent _deleteTriggerEvent;
        private bool _isPositionSet = false; 

        public void Update()
        {
            if (!_isPositionSet)
            {
                if (_text.bounds.extents.x > 0.0)
                {
                    UpdatePosition();
                    _isPositionSet = true;
                }
            }
        }
        public void UpdatePosition()
        {
            Bounds textBounds = _text.bounds;
            _button.transform.localPosition = new Vector3(textBounds.center.x + textBounds.extents.x + _labelSeparation, 0.0f, 0.0f);
        }

        public void TriggerDelete ()
        {
            _deleteTriggerEvent.Raise(_text.text);
        }
    }
}