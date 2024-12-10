using UnityEngine;
using ARInterface.Core.Variables;
using ARInterface.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace ARInterface.WorldMonitor
{
    public class ObjectPoseUpdater : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging;
        [SerializeField] private StringArrayVariable _detectionClasses;
        [SerializeField] private Detection3DArrayVariable _detections;
        [SerializeField] private WorldObjectDictionary _worldObjectDictionary; 

        private void Update()
        {
            if(_detections.Value != null)
            {
                foreach (var detection in _detections.Value.detections)
                {
                    var name = _detectionClasses.Values[detection.results[0].id];
                    if (_worldObjectDictionary.Items.TryGetValue(name, out var obj))
                    {
                        var (position, rotation) = (detection.bbox.center.position.From<FLU>(), detection.bbox.center.orientation.From<FLU>());
                        obj.transform.SetLocalPositionAndRotation(position, rotation);
                    }
                }
            }
        }

        private void Log(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[ObjectPoseTracker] " + msg);
        }

        private void LogWarning(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[ObjectPoseTracker] " + msg);
        }

        private void LogError(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[ObjectPoseTracker] " + msg);
        }
    }
}
