using UnityEngine;
using ARInterface.Core.Variables;

namespace ARInterface.HUD
{
    public class ToggleDeleteState : MonoBehaviour
    {
        [SerializeField] private BoolVariable _deleteState;

        public void Toggle()
        {
            _deleteState.Value = !_deleteState.Value;
        }

        private void OnDisable()
        {
            _deleteState.Value = false;
        }
    }
}