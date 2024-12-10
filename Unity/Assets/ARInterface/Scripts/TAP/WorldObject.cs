using ARInterface.Core;
using System.Collections.Generic;
using UnityEngine;

namespace ARInterface.TAP
{
    public class WorldObject : MonoBehaviour, IOnCollisionProxy
    {
        [SerializeField] private List<string> _collidingZones = new();

        public bool IsInsideZone(string zoneName)
        {
            return _collidingZones.Contains(zoneName);
        }

        public string GetName()
        {
            return name;
        }

        public void OnCollisionProxyEnter(Collider collision)
        {
            if (collision.gameObject.CompareTag("zone"))
            {
                var zoneName = collision.transform.parent.name;
                if (!_collidingZones.Contains(zoneName))
                {
                    _collidingZones.Add(zoneName);
                }
            }
        }
        public void OnCollisionProxyExit(Collider collision)
        {
            if (collision.gameObject.CompareTag("zone"))
            {
                var zoneName = collision.transform.parent.name;
                if (_collidingZones.Contains(zoneName))
                {
                    _collidingZones.Remove(zoneName);
                }
            }
        }
    }
}
