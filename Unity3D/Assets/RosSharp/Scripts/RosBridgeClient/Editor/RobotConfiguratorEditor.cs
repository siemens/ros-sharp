/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

using UnityEngine;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    [CustomEditor(typeof(RobotConfigurator))]
    public class RobotConfiguratorEditor : Editor
    {
        private RobotConfigurator robotConfigurator;
        private static GUIStyle buttonStyle;

        public override void OnInspectorGUI()
        {
            if (buttonStyle == null)
                buttonStyle = new GUIStyle(EditorStyles.miniButtonRight) { fixedWidth = 75 };

            robotConfigurator = (RobotConfigurator) target;

            GUILayout.Space(5);
            GUILayout.Label("All Rigidbodies", EditorStyles.boldLabel);
            DisplaySettingsToggle(new GUIContent("Is Kinematic"), robotConfigurator.SetRigidbodiesIsKinematic);
            DisplaySettingsToggle(new GUIContent("Use Gravity"), robotConfigurator.SetRigidbodiesUseGravity);
            DisplaySettingsToggle(new GUIContent("Use Inertia from URDF", "If disabled, Unity will generate new inertia tensor values automatically."), 
                robotConfigurator.SetUseUrdfInertiaData);

            GUILayout.Space(5);
            GUILayout.Label("All Colliders", EditorStyles.boldLabel);
            DisplaySettingsToggle(new GUIContent("Convex"), robotConfigurator.SetCollidersConvex);

            GUILayout.Space(5);
            GUILayout.Label("All Urdf Joints", EditorStyles.boldLabel);
            DisplaySettingsToggle(new GUIContent("Publish Joint State", "Adds/removes a Joint State Reader on each joint."),
                robotConfigurator.SetPublishJointStates);
            DisplaySettingsToggle(new GUIContent("Subscribe Joint State", "Adds/removes a Joint State Writer on each joint."),
                robotConfigurator.SetSubscribeJointStates);
        }

        private delegate void SettingsHandler(bool enable);

        private void DisplaySettingsToggle(GUIContent label, SettingsHandler handler)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel(label);
            if (GUILayout.Button("Enable", buttonStyle))
                handler(true);
            if (GUILayout.Button("Disable", buttonStyle))
                handler(false);
            EditorGUILayout.EndHorizontal();
        }
    }
}