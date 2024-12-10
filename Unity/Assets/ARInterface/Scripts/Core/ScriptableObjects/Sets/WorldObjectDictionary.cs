using UnityEngine;

namespace ARInterface.Core
{
    [CreateAssetMenu(menuName = "Sets/WorldObjectDictionary")]
    public class WorldObjectDictionary : RuntimeDictionary<string, GameObject>
    {
    }
}

