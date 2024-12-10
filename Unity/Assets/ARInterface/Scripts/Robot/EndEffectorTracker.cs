using UnityEngine;
using ARInterface.Core.Variables;

namespace ARInterface.Robot
{
    public class EndEffectorTracker : MonoBehaviour
    {
        // Spawn the starting pose of the reference gameobject
        [SerializeField] private GameObject _endEffectorPrefab;
        [SerializeField] private Transform _parentFrame;
        [SerializeField] private PoseVariable _endEffectorPose; 

        private GameObject _manipulableGripper; 

        void Start()
        {
            GameObject poseRef = Instantiate(_endEffectorPrefab, _parentFrame);
            poseRef.GetComponent<HandTracker>().SetEETransform(transform);
            _manipulableGripper = poseRef.transform.GetChild(0).gameObject;
        }

        private void Update()
        {
            var pos = GetEndEffectorPosition();
            _endEffectorPose.Position = pos.Item1;
            _endEffectorPose.Rotation = pos.Item2;
        }

        private (Vector3, Quaternion) GetEndEffectorPosition()
        {
            Vector3 eePos = _parentFrame.transform.InverseTransformPoint(_manipulableGripper.transform.position);
            Quaternion eeRot = (Quaternion.Inverse(_parentFrame.transform.rotation) * _manipulableGripper.transform.rotation);

            return (eePos, eeRot);
        }
    }
}
