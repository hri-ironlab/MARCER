using UnityEngine;

namespace ARInterface.Core
{
    public interface IOnCollisionProxy
    {
        public void OnCollisionProxyEnter(Collider collision);
        public void OnCollisionProxyExit(Collider collision);
    }

}
