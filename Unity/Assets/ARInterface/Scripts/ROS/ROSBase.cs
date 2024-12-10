using ARInterface.Core.Variables;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace ARInterface.ROS
{

    public abstract class ROSBase : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging;

        protected ROSConnection _ROS;

        // Start is called before the first frame update
        private void Awake()
        {
            _ROS = ROSConnection.GetOrCreateInstance();
        }
        protected void Log(string name, string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[" + name + "] " + msg);
        }
    }
}
