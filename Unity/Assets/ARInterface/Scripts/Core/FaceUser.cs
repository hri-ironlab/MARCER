using UnityEngine;

namespace ARInterface.Core
{
    public class FaceUser : MonoBehaviour
    {
        // Update is called once per frame
        void Update()
        {
            transform.rotation = Camera.main.transform.rotation;
        }
    }
}