using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.Moveit;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

namespace ARInterface.WorldMonitor
{
    public class InitializeViconToHololens : MonoBehaviour
    {
        [SerializeField] ModelStatesVariable _modelStates;
        [SerializeField] GameObject odom;
        [SerializeField] string _originFrameName = "base_link";
        [SerializeField] Event setWorldOriginEvent;
        private bool _isSceneInitialized = false;
        private PoseMsg _poseSamples = new PoseMsg();
        private int _numSamples = 5000; 

        private void Start()
        {
            
        }

        // Update is called once per frame
        private void Update()
        {
            if (!_isSceneInitialized && _modelStates.ModelStates.name != null)
            {
                if (_modelStates.ModelStates.name.Length > 0)
                {
                    InitializeScene();
                    _isSceneInitialized = true;
                    enabled = false; 
                }
            }
        }

        private void InitializeScene()
        {
            for(int i = 0; i < _modelStates.ModelStates.name.Length; i++)
            {

                if (_modelStates.ModelStates.name[i] == _originFrameName)
                {

                    for (int k = 0; k < _numSamples; k++)
                    {
                        _poseSamples.position.x += _modelStates.ModelStates.pose[i].position.x;
                        _poseSamples.position.y += _modelStates.ModelStates.pose[i].position.y;
                        _poseSamples.position.z += _modelStates.ModelStates.pose[i].position.z;
                        _poseSamples.orientation.x += _modelStates.ModelStates.pose[i].orientation.x;
                        _poseSamples.orientation.y += _modelStates.ModelStates.pose[i].orientation.y;
                        _poseSamples.orientation.z += _modelStates.ModelStates.pose[i].orientation.z;
                        _poseSamples.orientation.w += _modelStates.ModelStates.pose[i].orientation.w;
                    }
                    _poseSamples.position.x /= _numSamples;
                    _poseSamples.position.y /= _numSamples;
                    _poseSamples.position.z /= _numSamples;
                    _poseSamples.orientation.x /= _numSamples;
                    _poseSamples.orientation.y /= _numSamples;
                    _poseSamples.orientation.z /= _numSamples;
                    _poseSamples.orientation.w /= _numSamples;

                    odom.transform.parent = Camera.main.transform;
                    odom.transform.SetLocalPositionAndRotation(_poseSamples.position.From<FLU>(), _poseSamples.orientation.From<FLU>());
                    odom.transform.parent = null;
                }
            }
        }

        public void OnEnable()
        {
            _isSceneInitialized = false; 
        }
    }

}