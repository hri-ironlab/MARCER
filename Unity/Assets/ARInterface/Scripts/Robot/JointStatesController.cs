using System.Collections.Generic;
using UnityEngine;
using ARInterface.Core.Variables; 

namespace ARInterface.Robot
{
    public class JointStatesController : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging; 
        private Dictionary<string, JointController> _jointDict = new();

        // Start is called before the first frame update
        void Start()
        {            
            var jointControllers = GetComponentsInChildren<JointController>();
            foreach (var jointController in jointControllers)
            {
                _jointDict.Add(jointController.GetJointName(), jointController);
            }
        }

        public void SetJointPositions(string[] jointNames, double[] jointPosition)
        {
            if (jointNames != null)
            {
                int jointCount = jointNames.Length;

                for (int i = 0; i < jointCount; i++)
                {
                    string jointName = jointNames[i];

                    if (!_jointDict.TryGetValue(jointName, out var joint) || joint == null)
                    {
                        Log($"Joint reference for '{jointName}' does not exist.");
                        continue; 
                    }

                    // Set joint position
                    _jointDict[jointName].SetJointPosition((float)jointPosition[i]);
                }
            }
        }

        private void Log(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[JointStatesController] " + msg);
        }
    }
}