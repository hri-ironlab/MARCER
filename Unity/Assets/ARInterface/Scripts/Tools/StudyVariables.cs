using UnityEngine;
using ARInterface.Core.Variables;
using ARInterface.Core.Events;
using System.IO;
using System.Collections.Generic;

namespace ARInterface.Tools
{
    public class StudyVariables : MonoBehaviour, IStringResponse
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private string participant;
        [SerializeField] private StringVariable _zoneSaveParticipantID;
        [SerializeField] private StringEvent _loggingEvent;
        [SerializeField] private IntegerVariable _persistantIncrementer;
        private string _loggingFileName; 
        private List<StudyData> studyDataList = new List<StudyData>();

        [System.Serializable]
        public class StudyData
        {
            public string logMessage;
            public string timestamp; // Added field for timestamp
        }

        [System.Serializable]
        private class StudyDataContainer
        {
            public StudyData[] data;
        }
        private void Start()
        {
            SetSaveVariableName(); 
            _persistantIncrementer.Value = _persistantIncrementer.Value + 1;
        }

        private void SetSaveVariableName()
        {
            _zoneSaveParticipantID.Value = participant;
            _loggingFileName = Application.dataPath + "/UserData/LoggingData/" + participant + "_" + _persistantIncrementer.Value.ToString() + ".json";
        }

        public void OnStringResponse(string response)
        {
            // Capture the response with a timestamp and save it
            StudyData data = new StudyData
            {
                logMessage = response,
                timestamp = System.DateTime.UtcNow.ToString("hh:mm:ss tt")
            };
            studyDataList.Add(data);
        }


        private void SaveData()
        {
            // Convert the List<StudyData> to an array for serialization
            StudyDataContainer container = new StudyDataContainer
            {
                data = studyDataList.ToArray()
            };

            string json = JsonUtility.ToJson(container, true); // Pretty print the JSON
            File.WriteAllText(_loggingFileName, json);
        }


        private void OnEnable()
        {
            _loggingEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _loggingEvent.UnregisterListener(this);
        }

        private void OnApplicationQuit()
        {
            SaveData(); // Save data after logging the response
        }
    }
}
