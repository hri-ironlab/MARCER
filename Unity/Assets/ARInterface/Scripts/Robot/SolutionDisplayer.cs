using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.MoveitTaskConstructor;
using System.Collections;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Manipulation;
using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Security.Cryptography;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using ARInterface.TAP;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.Robot
{
    public class SolutionDisplayer : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging; 
        [SerializeField] private SolutionVariable _solution;
        [SerializeField] private DisplaySolutionVariable _displaySolution;
        [SerializeField] private DispatchActionsActionFeedbackVariable _feedback;
        [SerializeField] private ManipulationPlanRequestVariable _manipulationPlanRequestVariable; 
        [SerializeField] private JointsController _jointsController;
        [SerializeField] private JointStatesController _jointStatesController;

        [SerializeField] private List<GameObject> _initialStateObjects;
        [SerializeField] private List<GameObject> _finalStateObjects;
        [SerializeField] private Transform _attachObjectLink;
        [SerializeField] private Transform _objectWorldLink;
        [SerializeField] private List<GameObject> _surfaces;
        private GameObject _activeSurface; 
        private GameObject _initialStateObject;
        private GameObject _finalStateObject;
        private bool _isAttached; 

        [SerializeField] private float _jointAssignmentDelay = 1f;
        [SerializeField] private float _fingerAssignmentDelay = .1f;
        [SerializeField] private float _poseAssignmentDelay = 0.3f;
        [SerializeField] private int trajectoryToUse = 2;
        [SerializeField] private bool _isDisplayingTrajectory = false;
        [SerializeField] private int _displayingTaskID = -1;
        private IEnumerator coroutine;
        [SerializeField] private int maxSolutionDisplays = 1; 
        private int solutionDisplaysCount = 0; 

        // Update is called once per frame
        void Update()
        {
            if (_displaySolution.Solution != null )
            {
                if (!_isDisplayingTrajectory && (solutionDisplaysCount != maxSolutionDisplays))
                {
                    CheckAndDisplaySolution();
                }
                else
                {
                    CheckForNewTrajectory();
                }
            }
        }

        private void CheckForNewTrajectory()
        {
            if (_displayingTaskID != _displaySolution.Solution.task_id)
            {
                StopCoroutine(coroutine);
                solutionDisplaysCount = 0; 
                DisableDisplayedGameObjects();
                _isDisplayingTrajectory = false;
            }
        }

        private void CheckAndDisplaySolution()
        {
            if (_manipulationPlanRequestVariable.Request != null)
            {
                if (_manipulationPlanRequestVariable.Request.task_type == ManipulationPlanRequestMsg.DONE)
                {
                    _displaySolution.Clear();
                    _jointsController.enabled = true;
                    if (solutionDisplaysCount == maxSolutionDisplays)
                    {
                        DisableDisplayedGameObjects();
                    }
                    return;
                }
            }
            _jointsController.enabled = false;
            _displayingTaskID = _displaySolution.Solution.task_id;
            coroutine = DisplaySolution();
            StartCoroutine(coroutine);
        }
        
        private GameObject GetFinalStateObjectByName(string name)
        {
            foreach (GameObject gameObject in _finalStateObjects)
            {
                if(gameObject.name == name)
                {
                    return gameObject;
                }
            }
            return null; 
        }
        private GameObject GetInitialStateObjectByName(string name)
        {
            foreach (GameObject gameObject in _initialStateObjects)
            {
                if (gameObject.name == name)
                {
                    return gameObject;
                }
            }
            return null;
        }
        private void SetupScene()
        {
            var start_scene = _displaySolution.Solution.start_scene;

            // check if any object is placed somewhere in the world, then set its copy there
            if (start_scene.future_placed_object_end_pose.header.frame_id != "")
            {
                _finalStateObject = GetFinalStateObjectByName(start_scene.future_placed_object_end_pose.header.frame_id);
                _finalStateObject.SetActive(true);
                SetFinalStatePosition(start_scene.future_placed_object_end_pose.pose);
            }

            // If a surface is referenced in the command, highlight it
            foreach (GameObject surface in _surfaces)
            {
                if (_manipulationPlanRequestVariable.Request.target_object_name.Contains(surface.name))
                {
                    foreach (var start_surface in start_scene.involved_locations)
                        if (start_surface.header.frame_id == surface.name)
                        {
                            surface.transform.SetLocalPositionAndRotation(start_surface.pose.position.From<FLU>(), start_surface.pose.orientation.From<FLU>());
                            surface.SetActive(true);
                            _activeSurface = surface;
                        }
                }
                else
                {
                    surface.SetActive(false);
                }
            }
          

            // if an object is attached at the start of the scene, set the initial state to the robots hand
            if (start_scene.current_attached_object_start_pose.header.frame_id != "")
            {
                _initialStateObject = GetInitialStateObjectByName(start_scene.current_attached_object_start_pose.header.frame_id);
                _initialStateObject.SetActive(true);
                SetInitialAttachedObjectState();
            }
            else if(start_scene.future_attached_object_start_pose.header.frame_id != "")
            {
                _initialStateObject = GetInitialStateObjectByName(start_scene.future_attached_object_start_pose.header.frame_id);
                _initialStateObject.SetActive(true);
                _initialStateObject.transform.SetLocalPositionAndRotation(start_scene.future_attached_object_start_pose.pose.position.From<FLU>(), start_scene.future_attached_object_start_pose.pose.orientation.From<FLU>());
            }
        }

        private void SetInitialAttachedObjectState()
        {
            var initialPose = _displaySolution.Solution.start_scene.current_attached_object_start_pose.pose;
            _initialStateObject.transform.SetLocalPositionAndRotation(initialPose.position.From<FLU>(), initialPose.orientation.From<FLU>());
            _initialStateObject.transform.SetParent(_attachObjectLink.transform);
            _isAttached = true; 
        }


        private void SetFinalStatePosition(PoseMsg pose)
        {
            _finalStateObject.transform.SetLocalPositionAndRotation(pose.position.From<FLU>(), pose.orientation.From<FLU>());
        }

        private IEnumerator DisplaySolution()
        {
            _isDisplayingTrajectory = true;
            if (_displaySolution.Solution.sub_trajectories.Length > 0)
            {
                SetupScene();
                // For every sub trajectory
                foreach (MinimalSubTrajectoryMsg subTraj in _displaySolution.Solution.sub_trajectories)
                {
                    if (subTraj.joint_names != null)
                    {
                        UpdateAttachedObject(subTraj);
                        // For every robot pose in sub trajectory plan
                        foreach (var point in subTraj.positions)
                        {
                            float delay = _jointAssignmentDelay; 
                            var dispTraj = point.positions;
                            if(subTraj.joint_names.Length == 2)
                            {
                                delay = _fingerAssignmentDelay;
                                dispTraj[1] = -dispTraj[1];
                            }
                            _jointStatesController.SetJointPositions(subTraj.joint_names, dispTraj);
                            // Wait for robot to achieve pose for all joint assignments
                            yield return new WaitForSeconds(delay);
                        }
                        // Wait for the robot to achieve the final pose from joint assignment
                        yield return new WaitForSeconds(_poseAssignmentDelay);
                    }
                }
            }
            solutionDisplaysCount+=1;
            _isDisplayingTrajectory = false; 
        }

        private void UpdateAttachedObject(MinimalSubTrajectoryMsg subTrajectoryMsg)
        {
            if (subTrajectoryMsg.object_state_change == MinimalSubTrajectoryMsg.OBJECT_DETACHED)
            {
                _initialStateObject.transform.SetParent(_objectWorldLink.transform);
            }
            else if (subTrajectoryMsg.object_state_change == MinimalSubTrajectoryMsg.OBJECT_ATTACHED)
            {
                _initialStateObject.transform.SetParent(_attachObjectLink.transform);
            }
        }

        private void DisableDisplayedGameObjects()
        {
            if (_finalStateObject != null)
            {
                _finalStateObject.SetActive(false);
            }
            if (_initialStateObject != null)
            {
                _initialStateObject.SetActive(false);
            }
            if(_activeSurface != null)
            {
                _activeSurface.SetActive(false);
            }
        }

        private void Log(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[SolutionDisplayer] " + msg);
        }

        private void OnDisable()
        {
            _solution.Clear();
            _displaySolution.Clear();

        }
    }
}