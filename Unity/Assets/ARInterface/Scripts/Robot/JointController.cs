using UnityEngine;

namespace ARInterface.Robot
{
    public class JointController : MonoBehaviour
    {
        public enum JointType { Prismatic, Continuous, Revolute };
        public enum MovementAxis { x, y, z };

        [SerializeField] private JointType _jointType;
        [SerializeField] private MovementAxis _axis;
        [SerializeField] private float _lowerLimit;
        [SerializeField] private float _upperLimit;
        [SerializeField] private Vector3 _prismaticStartPos;
        [SerializeField] private Vector3 _rotationStartDeg;

        private Transform _jointTransform;
        private string _jointName;

        private void Awake()
        {
            _jointName = name.Replace("link", "joint");
            _jointTransform = gameObject.transform;
            _prismaticStartPos = _jointTransform.localPosition;
            _rotationStartDeg = _jointTransform.localEulerAngles;
        }
        public string GetJointName()
        {
            return _jointName;
        }

        public void SetJointPosition(float jointValue)
        {
            if (_jointType == JointType.Prismatic)
            {
                SetJointPositionPrismatic(ClampValue(jointValue));
            }
            else if (_jointType == JointType.Revolute)
            {
                SetJointRotation(ClampValue(jointValue));
            }
            else
            {
                SetJointRotation(jointValue);
            }
        }

        private void SetJointPositionPrismatic(float jointValue)
        {
            if (_axis == MovementAxis.x)
            {
                _jointTransform.localPosition = new Vector3(_prismaticStartPos.x + jointValue, _prismaticStartPos.y, _prismaticStartPos.z);
            }
            else if (_axis == MovementAxis.y)
            {
                _jointTransform.localPosition = new Vector3(_prismaticStartPos.x, _prismaticStartPos.y + jointValue, _prismaticStartPos.z);
            }
            else
            {
                _jointTransform.localPosition = new Vector3(_prismaticStartPos.x, _prismaticStartPos.y, _prismaticStartPos.z + jointValue);
            }
        }

        private void SetJointRotation(float jointValue)
        {
            if (_axis == MovementAxis.x)
            {
                _jointTransform.localRotation = Quaternion.Euler(_rotationStartDeg.x + jointValue * Mathf.Rad2Deg, _rotationStartDeg.y, _rotationStartDeg.z);
            }
            else if (_axis == MovementAxis.y)
            {
                _jointTransform.localRotation = Quaternion.Euler(_rotationStartDeg.x, _rotationStartDeg.y + (-jointValue) * Mathf.Rad2Deg, _rotationStartDeg.z);
            }
            else
            {
                _jointTransform.localRotation = Quaternion.Euler(_rotationStartDeg.x, _rotationStartDeg.y, _rotationStartDeg.x + (-jointValue) * Mathf.Rad2Deg);
            }
        }

        private float ClampValue(float jointValue)
        {
            if (jointValue > _upperLimit)
            {
                return _upperLimit;
            }
            else if (jointValue < _lowerLimit)
            {
                return _lowerLimit;
            }

            return jointValue;
        }
    }
}
