using ARInterface.Core.Variables;
using RosMessageTypes.Moveit; 
using UnityEngine;
using RosMessageTypes.Shape;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using ARInterface.Core;
using MixedReality.Toolkit;

namespace ARInterface.WorldMonitor
{
    public class WorldStateInitializer : MonoBehaviour
    {
        [SerializeField] private bool _enableDebugging;
        [SerializeField] private PlanningSceneWorldVariable _planningSceneWorldVariable;
        [SerializeField] private WorldObjectDictionary _worldObjectDictionary;
        [SerializeField] private GameObjectSet _activeLabels;

        [SerializeField] private GameObject _objectBasePrefab;
        [SerializeField] private GameObject _surfaceBasePrefab;
        [SerializeField] private GameObject _boxSurfacePrefab;
        [SerializeField] private GameObject _cylinderSurfacePrefab;
        [SerializeField] private GameObject _boxObjectPrefab;
        [SerializeField] private GameObject _cylinderObjectPrefab;
        [SerializeField] private GameObject _labelPrefab;

        private bool _isSceneInitialized = false;
        private Dictionary<string, GameObject> _surfaces = new();
        private Dictionary<string, GameObject> _objects = new();

        private void Update()
        {
            if (!_isSceneInitialized && _planningSceneWorldVariable.PlanningSceneWorld != null)
            {
                if (_planningSceneWorldVariable.PlanningSceneWorld.collision_objects.Length > 0)
                {
                    UpdateScene();
                    _isSceneInitialized = true;
                }
            }
        }

        private void UpdateScene()
        {
            foreach (var collisionObject in _planningSceneWorldVariable.PlanningSceneWorld.collision_objects)
            {
                switch (collisionObject.operation)
                {
                    case CollisionObjectMsg.ADD:
                        AddCollisionObject(collisionObject);
                        break;
                    case CollisionObjectMsg.MOVE:
                        MoveCollisionObject(collisionObject);
                        break;
                    case CollisionObjectMsg.REMOVE:
                        RemoveCollisionObject(collisionObject.id);
                        break;
                    default:
                        Log($"Unhandled operation: {collisionObject.operation}");
                        break;
                }
            }
            Log("Scene Updated");
        }

        private void RemoveCollisionObject(string id)
        {
            if (_surfaces.TryGetValue(id, out GameObject surface))
            {
                Destroy(surface);
                _surfaces.Remove(id);
            }
            else if (_objects.TryGetValue(id, out GameObject _object))
            {
                Destroy(_object);
                _objects.Remove(id);
            }
            else
            {
                Log($"{id} collision object not found in scene.");
            }
        }

        private void MoveCollisionObject(CollisionObjectMsg collisionObject)
        {
            if (_objects.TryGetValue(collisionObject.id, out GameObject _object))
            {
                var (position, rotation) = (collisionObject.pose.position.From<FLU>(), collisionObject.pose.orientation.From<FLU>());
                _object.transform.SetLocalPositionAndRotation(position, rotation);
            }
            else
            {
                AddCollisionObject(collisionObject);
            }
        }

        private void AddCollisionObject(CollisionObjectMsg collisionObject)
        {
            string id = collisionObject.id.ToString();
            (Vector3 position, Quaternion rotation) = (collisionObject.pose.position.From<FLU>(), collisionObject.pose.orientation.From<FLU>());

            // Either use prefab for world objects with script on it or instantiate empty object for surfaces
            GameObject objectBase = collisionObject.type.key == "object" ? Instantiate(_objectBasePrefab) : Instantiate(_surfaceBasePrefab);
            objectBase.transform.SetParent(transform);
            objectBase.name = id; 
            objectBase.transform.SetLocalPositionAndRotation(position, rotation);

            for (int i = 0; i < collisionObject.primitives.Length; i++)
            {
                GameObject primitive = InstantiatePrimitive(collisionObject.primitives[i], collisionObject.type.key, objectBase.transform);

                if(i == 0)
                {
                    if(collisionObject.type.key != "collision_object"){
                        // Spawn the label for the trigger
                        GameObject label = Instantiate(_labelPrefab);
                        label.transform.SetParent(transform);
                        TimedLabelDisplay timedLabelDisplay = label.GetComponent<TimedLabelDisplay>();
                        timedLabelDisplay.SetProperties(primitive.transform, id);
                        _activeLabels.Add(label);
                        
                        var statefulInteractable = primitive.GetComponent<StatefulInteractable>();
                        statefulInteractable.IsGazeHovered.OnEntered.AddListener((float arg) => timedLabelDisplay.DisplayLabel());
                        statefulInteractable.IsGazeHovered.OnExited.AddListener((float arg) => timedLabelDisplay.DisplayLabel()); 
                    }

                }

                if (primitive != null)
                {
                    SetPrimitiveTransform(primitive, collisionObject.primitive_poses[i], collisionObject.primitives[i]);
                    _worldObjectDictionary.Add(id, objectBase);
                }
            }

            if (collisionObject.type.key == "object")
            {
                _objects.Add(id, objectBase);
            }
            else if (collisionObject.type.key == "surface" || collisionObject.type.key == "collision_object")
            {
                _surfaces.Add(id, objectBase);
            }
        }

        private GameObject InstantiatePrimitive(SolidPrimitiveMsg primitiveType, string objectType, Transform parent)
        {
            GameObject prefab = null;

            switch (primitiveType.type)
            {
                case SolidPrimitiveMsg.BOX:
                    prefab = objectType == "object" ? _boxObjectPrefab : _boxSurfacePrefab;
                    break;
                case SolidPrimitiveMsg.CYLINDER:
                    prefab = objectType == "object" ? _cylinderObjectPrefab : _cylinderSurfacePrefab;
                    break;
                default:
                    Log($"Unhandled primitive type: {primitiveType}");
                    break;
            }

            return prefab != null ? Instantiate(prefab, parent) : null;
        }

        private void SetPrimitiveTransform(GameObject primitive, PoseMsg primitivePose, SolidPrimitiveMsg primitiveData)
        {
            primitive.transform.SetLocalPositionAndRotation(primitivePose.position.From<FLU>(), primitivePose.orientation.From<FLU>());

            if (primitiveData.type == SolidPrimitiveMsg.BOX)
            {
                SetBoxScale(primitive, primitiveData.dimensions);
            }
            else if (primitiveData.type == SolidPrimitiveMsg.CYLINDER)
            {
                SetCylinderScale(primitive, primitiveData.dimensions);
            }
            else
            {
                Log($"Unhandled primitive type: {primitiveData.type}");
            }
        }

        private void SetBoxScale(GameObject primitive, double[] dimensions)
        {
            // Convert scale to Unity coordinate frame but x goes negative using From<FLU>()
            var surfaceScale = new PointMsg((float)dimensions[0], (float)dimensions[1], (float)dimensions[2]).From<FLU>();
            primitive.transform.localScale = new Vector3(Mathf.Abs(surfaceScale.x), Mathf.Abs(surfaceScale.y), Mathf.Abs(surfaceScale.z));
        }

        private void SetCylinderScale(GameObject primitive, double[] dimensions)
        {
            float radius = (float)dimensions[SolidPrimitiveMsg.CYLINDER_RADIUS] * 2.0f;
            float height = (float)dimensions[SolidPrimitiveMsg.CYLINDER_HEIGHT];
            primitive.transform.localScale = new Vector3(radius, height / 2.0f, radius);
        }

        private void OnApplicationQuit()
        {
            _planningSceneWorldVariable.Clear();
        }
        private void Log(string msg)
        {
            if (!_enableDebugging) return;
            Debug.Log("[WorldStateInitializer] " + msg);
        }
    }
}
