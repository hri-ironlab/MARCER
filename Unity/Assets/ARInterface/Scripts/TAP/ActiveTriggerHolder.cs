using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARInterface.Core; 

namespace ARInterface.TAP
{
public class ActiveTriggerHolder : MonoBehaviour
{
    public List<GameObject> _activeTriggers = new();


    public void Clear()
    {
        for (int i = 0; i < _activeTriggers.Count; i++)
        {
            var temp = _activeTriggers[i];
            _activeTriggers.Remove(temp);
            Destroy(temp);
        }
        _activeTriggers.Clear();
    }

    public void DestroyObjectByName(string name){
        for (int i = 0; i < _activeTriggers.Count; i++)
        {
            var temp = _activeTriggers[i];
            if(temp.name == name){
                _activeTriggers.Remove(temp);
                Destroy(temp);
                break;
            }  
        } 
    }
}
}
