using UnityEngine;
using System.Collections.Generic;
using ARInterface.Core;
using System;
using MixedReality.Toolkit;
using MixedReality.Toolkit.SpatialManipulation;
using UnityEngine.UI;

namespace ARInterface.TAP
{
    public class AllowedManipulationsSetter : MonoBehaviour
    {
        [SerializeField] private ObjectManipulator _objectManipulator;

        private bool moveAllowed = false;

        private void Start()
        {
            
        }

        public void ToggleAllowTranslation()
        {
            moveAllowed = !moveAllowed; 

            if (moveAllowed)
            {
                _objectManipulator.AllowedManipulations = TransformFlags.Move;
            }
            else
            {
                _objectManipulator.AllowedManipulations = TransformFlags.None;
            }
        }
    }
}