/*
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
using UrdfExport.UrdfComponents;

namespace UrdfExport.Editor.CustomComponentEditorss
{
    [CustomEditor(typeof(UrdfRobot))]
    public class UrdfRobotEditor : UnityEditor.Editor
    {
        private UrdfRobot urdfRobot;

        public override void OnInspectorGUI()
        {
            urdfRobot = (UrdfRobot)target;

            //GUILayout.Space(10);
            //if (GUILayout.Button("Initialize robot"))
            //    urdfRobot.InitializeRobot();

            GUILayout.Space(5);
            if (GUILayout.Button("Export robot to URDF file"))
                urdfRobot.ExportRobotToUrdf();
        }
    }
}
