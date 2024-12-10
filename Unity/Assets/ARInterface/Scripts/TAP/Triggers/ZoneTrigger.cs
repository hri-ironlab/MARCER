using UnityEngine;
using System.Collections.Generic;
using ARInterface.Core;
using System;

namespace ARInterface.TAP
{
    public class ZoneTrigger : TriggerBase, IOnCollisionProxy
    {
        [SerializeField] private List<string> _collidingObjects = new();
        [SerializeField] private double _checkUserLocationRate = 5f;
        private double _lastCheckTimeSeconds;
        private double CheckPeriodSeconds => 1.0f / _checkUserLocationRate;
        private bool ShouldCheckUserLocation => Clock.FrameStartTimeInSeconds - CheckPeriodSeconds > _lastCheckTimeSeconds;
        private Transform _childTransform;

        private void Start()
        {
            _childTransform = gameObject.transform.GetChild(0).transform;
        }
        private void Update()
        {
/*            if (ShouldCheckUserLocation)
            {
                CheckUserLocation();
            }*/
        }

        public string GetName()
        {
            return name;
        }

        public Vector3 GetPlacePosition()
        {
            return transform.position;
        }

        public void CheckUserLocation()
        {
            var checkTime = Clock.time;
            Vector3 cameraPosition = Camera.main.transform.position;
            if (IsPointInsideZone(cameraPosition))
            {
                if(!_collidingObjects.Contains("User"))
                {
                    _collidingObjects.Add("User");
                }
            }
            else
            {
                if (_collidingObjects.Contains("User"))
                {
                    _collidingObjects.Remove("User");
                }
            }
            _lastCheckTimeSeconds = checkTime;
        }
        public bool IsPointInsideZone(Vector3 pos)
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


        public void OnCollisionProxyEnter(Collider collision)
        {            
            // Retrieve the name of the world object associated with the collider
            string name = collision.transform.parent.name;
            if (collision.gameObject.CompareTag("world object"))
            {
                if (!_collidingObjects.Contains(name))
                {
                    _collidingObjects.Add(name);
                }
            }
        }

        public void OnCollisionProxyExit(Collider collision) 
        {
            // Retrieve the name of the world object associated with the collider
            string name = collision.transform.parent.name;
            if (collision.gameObject.CompareTag("world object"))
            {
                // Remove the name from the list of colliding boxes
                if (_collidingObjects.Contains(name))
                {
                    _collidingObjects.Remove(name);
                }
            }
        }

        public List<string> GetObjectsInside()
        {
            return _collidingObjects;
        }

        public bool HasObjectsInside()
        {
            return _collidingObjects.Count > 0;
        }
        public bool HasObjectInside(string boxName)
        {
            return _collidingObjects.Contains(boxName);
        }
    }
}