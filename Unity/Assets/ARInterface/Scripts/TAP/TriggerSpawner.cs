using System.Collections.Generic;
using System.IO;
using UnityEngine;
using ARInterface.Core;
using MixedReality.Toolkit;
using ARInterface.Core.Events;
using System.Runtime.Serialization;
using MixedReality.Toolkit.SpatialManipulation;
using System.Text.RegularExpressions;
using ARInterface.TAP;
using log4net.Util;
using ARInterface.Core.Variables;

namespace ARInterface.TAP
{
    public class TriggerSpawner : MonoBehaviour, IStringResponse
    {
        [SerializeField] private bool _enableDebugging;
        [SerializeField] private bool _destroyActiveTriggersOnExit;
        [SerializeField] private GameObject _spawnParent; 
        [SerializeField] private GameObjectSet _triggerPrefabs;
        [SerializeField] private GameObject _labelPrefab;
        [SerializeField] private GameObjectSet _activeTriggers;
        [SerializeField] private GameObjectSet _activeLabels;
        [SerializeField] private StringEvent _onTriggerDelete;
        [SerializeField] private StringEvent _deleteTriggerEvent;
        [SerializeField] private ActiveTriggerHolder _activeTriggerHolder;
        [SerializeField] private StringVariable zoneSaveParticipantID; 
        private int _nextTriggerIndex = 1;
        List<int> deletedTriggerIndexes = new List<int>();

        private string _saveFilePath;
        private void Start()
        {
            //_saveFilePath = Application.dataPath + "/UserData/ZoneData/" + zoneSaveParticipantID.Value + ".json";
            //LoadTriggers(); // Load triggers when the game starts
        }
        public void SpawnTrigger()
        {
            // Get the camera's forward direction
            Vector3 cameraForward = Camera.main.transform.forward;

            // Calculate the position for spawning the object
            Vector3 spawnPosition = Camera.main.transform.position + cameraForward * 0.5f;

            // Spawn the trigger
            GameObject triggerBase = _triggerPrefabs.Items[0];
            GameObject triggerPrefab = Instantiate(triggerBase, spawnPosition, Quaternion.identity);    
            triggerPrefab.transform.SetParent(_spawnParent.transform);

            string triggerName = triggerBase.GetComponent<TriggerBase>().TriggerName;
            string labelText; 
            if(deletedTriggerIndexes.Count > 0 )
            {
                labelText = $"{triggerName} {deletedTriggerIndexes[0]}";
                deletedTriggerIndexes.RemoveAt(0);
            }
            else
            {
                labelText = $"{triggerName} {_nextTriggerIndex}";
                _nextTriggerIndex++;
            }
            triggerPrefab.name = labelText;
            _activeTriggers.Add(triggerPrefab);
           _activeTriggerHolder._activeTriggers.Add(triggerPrefab); 
            // Spawn the label for the trigger
            GameObject label = Instantiate(_labelPrefab);
            label.transform.SetParent(transform);
            TimedLabelDisplay timedLabelDisplay = label.GetComponent<TimedLabelDisplay>();
            timedLabelDisplay.SetProperties(triggerPrefab.transform, labelText);
            _activeLabels.Add(label);

            var statefulInteractable = triggerPrefab.GetComponent<StatefulInteractable>();
            statefulInteractable.IsGazeHovered.OnEntered.AddListener((float arg) => timedLabelDisplay.DisplayLabel());
            statefulInteractable.IsGazeHovered.OnExited.AddListener((float arg) => timedLabelDisplay.DisplayLabel());
        }

        public void OnStringResponse(string response)
        {
            deletedTriggerIndexes.Add(ExtractNumberFromEnd(response));
            deletedTriggerIndexes.Sort(); 
            DestroyLabel(response);
            DestroyTrigger(response); 
        }

        static int ExtractNumberFromEnd(string input)
        {
            Match match = Regex.Match(input, @"\d+$");
            if (match.Success)
            {
                return int.Parse(match.Value);
            }
            else
            {
                throw new System.Exception("No number found in zone name");
            }
        }
        private void DestroyLabel(string name)
        {
            GameObject label = GetLabelByName(name);
            if (label != null)
            {
                _activeLabels.Remove(label);
                Destroy(label);
            }
        }

        private GameObject GetLabelByName(string name)
        {
            foreach (GameObject label in _activeLabels.Items)
            {
                if (label.name == name)
                {
                    return label;
                }
            }
            return null; 
        }

        private void DestroyTrigger(string name)
        {
            _activeTriggerHolder.DestroyObjectByName(name); 
            _activeTriggers.DestroyObjectByName(name);
            DestroyLabel(name);
        }
        public void OnEnable()
        {
            _onTriggerDelete.RegisterListener(this);
        }
        private void OnDisable()
        {
            if (_destroyActiveTriggersOnExit)
            {
                _activeTriggers.Clear();
                _activeLabels.Clear(); 
            }
            _onTriggerDelete.UnregisterListener(this);
        }


        // TriggerData class to hold position, rotation, and name of the triggers
        [System.Serializable]
        private class TriggerData
        {
            public string name;
            public Vector3 position;
            public Vector3 rotation;
            public Vector3 scale;
        }

        // Updated TriggerDataWrapper class to hold additional info
        [System.Serializable]
        private class TriggerDataWrapper
        {
            public List<TriggerData> triggers;
            public int nextTriggerIndex;
            public List<int> deletedTriggerIndexes;
        }

        // Save all triggers' data to a JSON file
        private void SaveAllTriggers()
        {
            List<TriggerData> triggerDataList = new List<TriggerData>();

            foreach (GameObject trigger in _activeTriggers.Items)
            {
                TriggerData data = new TriggerData
                {
                    name = trigger.name,
                    position = trigger.transform.localPosition,
                    rotation = trigger.transform.localRotation.eulerAngles,
                    scale = trigger.transform.localScale
                };
                triggerDataList.Add(data);
            }

            // Save triggers, next trigger index, and deleted trigger indexes
            TriggerDataWrapper wrapper = new TriggerDataWrapper
            {
                triggers = triggerDataList,
                nextTriggerIndex = _nextTriggerIndex,
                deletedTriggerIndexes = new List<int>(deletedTriggerIndexes) // Save deleted indexes
            };

            string json = JsonUtility.ToJson(wrapper, true);
            File.WriteAllText(_saveFilePath, json);
        }
        // Load the saved triggers from a JSON file
        private void LoadTriggers()
        {
            if (File.Exists(_saveFilePath))
            {
                string json = File.ReadAllText(_saveFilePath);
                TriggerDataWrapper triggerDataWrapper = JsonUtility.FromJson<TriggerDataWrapper>(json);

                // Load saved triggers
                foreach (TriggerData data in triggerDataWrapper.triggers)
                {
                    GameObject triggerPrefab = Instantiate(_triggerPrefabs.Items[0], _spawnParent.transform);
                    triggerPrefab.transform.SetLocalPositionAndRotation(data.position, Quaternion.Euler(data.rotation));
                    triggerPrefab.transform.localScale = data.scale;
                    triggerPrefab.name = data.name;
                    _activeTriggers.Add(triggerPrefab);
                    _activeTriggerHolder._activeTriggers.Add(triggerPrefab);

                    GameObject label = Instantiate(_labelPrefab);
                    label.transform.SetParent(transform);
                    TimedLabelDisplay timedLabelDisplay = label.GetComponent<TimedLabelDisplay>();
                    timedLabelDisplay.SetProperties(triggerPrefab.transform, data.name);
                    _activeLabels.Add(label);
                    var statefulInteractable = triggerPrefab.GetComponent<StatefulInteractable>();
                    statefulInteractable.IsGazeHovered.OnEntered.AddListener((float arg) => timedLabelDisplay.DisplayLabel());
                    statefulInteractable.IsGazeHovered.OnExited.AddListener((float arg) => timedLabelDisplay.DisplayLabel());
                }

                // Restore next trigger index and deleted trigger indexes
                _nextTriggerIndex = triggerDataWrapper.nextTriggerIndex;
                deletedTriggerIndexes = new List<int>(triggerDataWrapper.deletedTriggerIndexes);
            }
        }
        private void OnApplicationQuit()
        {
            //SaveAllTriggers(); // Save triggers, index, and deleted indexes to JSON when quitting
        }
    }
}



