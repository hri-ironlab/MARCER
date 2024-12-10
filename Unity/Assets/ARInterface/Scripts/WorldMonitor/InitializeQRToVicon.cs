using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.Moveit;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace ARInterface.WorldMonitor
{
    public class InitializeQRToVicon : MonoBehaviour
    {
        [SerializeField] ModelStatesVariable _modelStates;
        [SerializeField] GameObject _ROSOriginAligner;
        [SerializeField] string _originFrameName = "base_link";
        [SerializeField] Event setWorldOriginEvent;
        private bool _isSceneInitialized = false;

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
                    var pose = _modelStates.ModelStates.pose[i];
                    _ROSOriginAligner.transform.SetLocalPositionAndRotation(pose.position.From<FLU>(), pose.orientation.From<FLU>());
                }
            }
        }

        public void OnEnable()
        {
            _isSceneInitialized = false; 
        }
    }

}