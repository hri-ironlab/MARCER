using UnityEngine;

namespace ARInterface.Core
{
    [CreateAssetMenu(menuName = "Sets/GameObjectSet"), System.Serializable]
    public class GameObjectSet : RuntimeSet<GameObject>
    {
        public void Clear()
        {
            for (int i = 0; i < Items.Count; i++)
            {
                var temp = Items[i];
                Items.Remove(temp);
                Destroy(temp);
            }
            Items.Clear();
        }

        public void DestroyObjectByName(string name){
            for (int i = 0; i < Items.Count; i++)
            {
                var temp = Items[i];
                if(temp.name == name){
                    Items.Remove(temp);
                    Destroy(temp);
                    break;
                }  
            } 
        }
    }
}
