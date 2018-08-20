﻿/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    [CustomEditor(typeof(UrdfJoint))]
    public class UrdfJointEditor : Editor
    {
        private UrdfJoint urdfJoint;

        public override void OnInspectorGUI()
        {
            urdfJoint = (UrdfJoint) target;
            
            urdfJoint.JointName = EditorGUILayout.TextField("Joint Name", urdfJoint.JointName);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel("Joint Type");
            EditorGUILayout.LabelField(urdfJoint.JointType.ToString());
            EditorGUILayout.EndHorizontal();
           
            switch (urdfJoint.JointType)
            {
                case UrdfJoint.JointTypes.Fixed:
                    break;
                case UrdfJoint.JointTypes.Continuous:
                    DisplayDynamicsMessage("HingeJoint > Spring > Damper (for damping) and Spring (for friction)");
                    DisplayAxisMessage("HingeJoint > Axis");
                    break;
                case UrdfJoint.JointTypes.Revolute:
                    DisplayDynamicsMessage("HingeJoint > Spring > Damper (for damping) and Spring (for friction)");
                    DisplayAxisMessage("HingeJoint > Axis");
                    DisplayRequiredLimitMessage("HingeJoint > Limits > Min / Max");
                    break;
                case UrdfJoint.JointTypes.Floating:
                    DisplayDynamicsMessage("ConfigurableJoint > xDrive > Position Damper (for Damping) and Position Spring (for friction)");
                    break;
                case UrdfJoint.JointTypes.Prismatic:
                    DisplayDynamicsMessage("ConfigurableJoint > xDrive > Position Damper (for Damping) and Position Spring (for friction)");
                    DisplayAxisMessage("ConfigurableJoint > Axis");
                    DisplayRequiredLimitMessage("ConfigurableJoint > Linear Limit > Limit");
                    break;
                case UrdfJoint.JointTypes.Planar:
                    DisplayDynamicsMessage("ConfigurableJoint > xDrive > Position Damper (for Damping) and Position Spring (for friction)");
                    DisplayAxisMessage("ConfigurableJoint > Axis and Secondary Axis");
                    DisplayRequiredLimitMessage("ConfigurableJoint > Linear Limit > Limit");
                    break;
            }
        }

        private void DisplayDynamicsMessage(string dynamicsLocation)
        {
            GUILayout.Space(5);
            EditorGUILayout.LabelField("Joint Dynamics (optional)", EditorStyles.boldLabel);

            EditorGUILayout.HelpBox("To define damping and friction values, edit the fields " + dynamicsLocation + ".", MessageType.Info);

        }

        private void DisplayAxisMessage(string axisLocation)
        {
            GUILayout.Space(5);
            
            EditorGUILayout.LabelField("Joint Axis", EditorStyles.boldLabel);

            EditorGUILayout.HelpBox("An axis is required for this joint type. Remember to define an axis in " + axisLocation + ".", MessageType.Info);
        }

        public void DisplayRequiredLimitMessage(string limitLocation)
        {
            GUILayout.Space(5);
            EditorGUILayout.LabelField("Joint Limits", EditorStyles.boldLabel);

            urdfJoint.effortLimit = EditorGUILayout.DoubleField("Effort Limit", urdfJoint.effortLimit);
            urdfJoint.velocityLimit = EditorGUILayout.DoubleField("Velocity Limit", urdfJoint.velocityLimit);

            if (!urdfJoint.AreLimitsCorrect())
                EditorGUILayout.HelpBox("Limits are required for this joint type. Please enter valid limit values in " + limitLocation + ".", MessageType.Warning);

            GUILayout.Space(5);
        }
    }
}