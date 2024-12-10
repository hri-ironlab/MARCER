using UnityEngine;
using ARInterface.Core.Variables;
using ARInterface.Core;
using System; 

namespace ARInterface.TAP
{
    public class UserLocationTrigger : TriggerBase
    {
        [SerializeField] FloatRangeVariable enableAlpha;
        [SerializeField] FloatRangeVariable disableAlpha;
        [SerializeField] private bool _isUserInside;
        [SerializeField] private double _checkRate = 5f;
        private double _lastCheckTimeSeconds;
        private double CheckPeriodSeconds => 1.0f / _checkRate;
        private bool ShouldCheckUserLocation => Clock.FrameStartTimeInSeconds - CheckPeriodSeconds > _lastCheckTimeSeconds;
        private Transform _childTransform; 

        private void Start()
        {
            SetTriggerType(1);
            SetTriggerName("User Location Trigger");

            _childTransform = gameObject.transform.GetChild(0).transform; 
        }

        private void Update()
        {
            if (ShouldCheckUserLocation)
            {
                CheckUseLocation();
            }
        }
        public void CheckUseLocation()
        {
            var checkTime = Clock.time;
            Vector3 cameraPosition = Camera.main.transform.position;
            _isUserInside = IsPointInside(cameraPosition);
            _lastCheckTimeSeconds = checkTime;
        }
        public bool IsPointInside(Vector3 pos)
        {
            var triggerTheta = -transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

            // Translate the point to the square's local coordinate system
            double translatedOriginX = pos.x - transform.position.x;
            double translatedOriginZ = pos.z - transform.position.z;

            // Rotate the translated point around the square's center by -theta
            double rotatedX = translatedOriginX * Math.Cos(triggerTheta) - translatedOriginZ * Math.Sin(triggerTheta);
            double rotatedZ = translatedOriginX * Math.Sin(triggerTheta) + translatedOriginZ * Math.Cos(triggerTheta);

            // Translate the rotated point back to the world coordinate system
            rotatedX += transform.position.x;
            rotatedZ += transform.position.z;

            // Check if the transformed point is within the square's bounds
            double minX = transform.position.x - _childTransform.localScale.x / 2;
            double maxX = transform.position.x + _childTransform.localScale.x / 2;
            double minZ = transform.position.z - _childTransform.localScale.z / 2;
            double maxZ = transform.position.z + _childTransform.localScale.z / 2;

            return minX <= rotatedX && rotatedX <= maxX && minZ <= rotatedZ && rotatedZ <= maxZ;
        }


        public UserLocationTrigger(Color color)
        {
            GetComponent<Renderer>().material.color = color;
        }

        public string GetName()
        {
            return name;
        }

        public bool IsTriggered()
        {
            return _isUserInside;
        }

        public void SetColor(Color color)
        {
            GetComponent<Renderer>().material.color = color;
        }
    }
}