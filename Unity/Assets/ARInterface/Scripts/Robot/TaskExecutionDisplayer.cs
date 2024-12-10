using UnityEngine;
using ARInterface.Core.Variables;
using System.Collections.Generic;
using RosMessageTypes.MoveitTaskConstructor;
using System.Collections;
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;

namespace ARInterface.Robot
{
    public class TaskExecutionDisplayer : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging;
        [SerializeField] private ExecuteTaskSolutionGoalVariable _goal;
        [SerializeField] private SolutionVariable _solution;


        // Update is called once per frame
        void Update()
        {
            if(_goal.Goal != null)
            {
                _solution.Solution = _goal.Goal.goal.solution; 
            }
        }

        private void Log(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[SolutionDisplayer] " + msg);
        }
    }
}