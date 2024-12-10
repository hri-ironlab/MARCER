using System.Collections.Generic;
using UnityEngine;
using ARInterface.Core.Variables; 

namespace ARInterface.Robot
{
    public class JointsController : MonoBehaviour
    {
        [SerializeField] private StringVariable _activeTopicName; 
        [SerializeField] private List<JointStatesVariable> _jointStateVariables;

        private Dictionary<string, JointController> _jointDict = new();
        private JointStatesVariable _activeJointStatesVariable;

        // Start is called before the first frame update
        void Start()
        {
            SetActiveSubscriber(_activeTopicName.Value);
            
            var jointControllers = GetComponentsInChildren<JointController>();
            foreach (var jointController in jointControllers)
            {
                _jointDict.Add(jointController.GetJointName(), jointController);
            }
        }

        // Update is called once per frame
        void Update()
        {
            SetJointAngles();
        }

        public void SetActiveSubscriber(string activeTopicName)
        {
            foreach (JointStatesVariable jointStatesVariable in _jointStateVariables)
            {
                if (jointStatesVariable.name.Contains(activeTopicName))
                {
                    _activeJointStatesVariable = jointStatesVariable;
                    break;
                }
            }
        }

        private void SetJointAngles()
        {
            var jointStates = _activeJointStatesVariable.JointStates;

            if (jointStates != null)
            {
                int jointCount = jointStates.name.Length;

                for (int i = 0; i < jointCount; i++)
                {
                    string jointName = jointStates.name[i];

                    if (!_jointDict.ContainsKey(jointName))
                    {
                        continue; 
                    }

                    if (_jointDict[jointName] == null)
                    {
                        Debug.LogError($"[JointsController]: Joint reference for '{jointName}' is null.");
                        continue; 
                    }

                    // Set joint position
                    _jointDict[jointName].SetJointPosition((float)jointStates.position[i]);
                }
            }
        }
    }
}