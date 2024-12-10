using MixedReality.Toolkit.Subsystems;
using UnityEngine;
using System.Collections;
using MixedReality.Toolkit;
using UnityEngine.XR;

namespace ARInterface.Robot
{
    public class HandTracker : MonoBehaviour
    {
        [SerializeField] private Vector3 _handPositionOffset;
        [SerializeField] private Quaternion _handRotationOffset;

        private HandsAggregatorSubsystem _handsSubsystem;
        private Transform _defaultTrackedTransform;
        private Transform _gripperTransform; 

        private void Start()
        {
            _gripperTransform = transform.GetChild(0);
            _handsSubsystem = XRSubsystemHelpers.GetFirstRunningSubsystem<HandsAggregatorSubsystem>();
            // Wait until an aggregator is available.
            if (_handsSubsystem == null)
            {
                StartCoroutine(EnableWhenSubsystemAvailable());
            }
        }

        public void SetEETransform(Transform defaultTrackedTransform)
        {
            _defaultTrackedTransform = defaultTrackedTransform;
        }

        /// <summary>
        /// Coroutine to wait until subsystem becomes available.
        /// </summary>
        private IEnumerator EnableWhenSubsystemAvailable()
        {
            yield return new WaitUntil(() => XRSubsystemHelpers.GetFirstRunningSubsystem<HandsAggregatorSubsystem>() != null);
        }

        private void Update()
        {
            if (_handsSubsystem.TryGetJoint(TrackedHandJoint.Palm, XRNode.RightHand, out HandJointPose jointPose))
            {
                _gripperTransform.SetLocalPositionAndRotation(_handPositionOffset, _handRotationOffset);
                transform.SetPositionAndRotation(jointPose.Position, jointPose.Rotation);
            }
            else
            {
                _gripperTransform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
                transform.SetPositionAndRotation(_defaultTrackedTransform.position, _defaultTrackedTransform.rotation);
            }
        }
    }
}