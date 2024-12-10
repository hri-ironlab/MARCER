using UnityEngine;

namespace ARInterface.Core
{
    public class ParentCollisionEventProxy : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging;
        private IOnCollisionProxy _collisionProxy;
        private void Start()
        {
            FindAndSetCollisionProxy();
        }
        private void FindAndSetCollisionProxy()
        {
            // Attempt to get the IOnCollisionProxy component from the parent object
            if (transform.parent.TryGetComponent(out _collisionProxy) == false)
            {
                Log("Add IOnCollisionProxy to the parent object of " + gameObject.name);
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            // Check if trigger is not null before calling ChildTriggerEnter
            _collisionProxy?.OnCollisionProxyEnter(other);
        }

        private void OnTriggerExit(Collider other)
        {
            // Check if trigger is not null before calling ChildTriggerExit
            _collisionProxy?.OnCollisionProxyExit(other);
        }

        private void Log(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[ParentEventProxy] " + msg);
        }
    }
}
