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

namespace RosSharp.Urdf.Export
{
    [CustomEditor(typeof(UrdfLink))]
    public class UrdfLinkEditor : UnityEditor.Editor
    {
        private UrdfLink urdfLink;
        private UrdfJoint.JointTypes jointType = UrdfJoint.JointTypes.Fixed;

        public override void OnInspectorGUI()
        {
            urdfLink = (UrdfLink)target;

            GUILayout.Space(10);
            if (GUILayout.Button("Add child link"))
                urdfLink.AddChildLink();

            GUILayout.Space(5);
            jointType = (UrdfJoint.JointTypes) EditorGUILayout.EnumPopup(
                "Type of joint", jointType);
            if (GUILayout.Button("Add child link with connecting joint"))
                urdfLink.AddChildLink(jointType);
        }
    }
}
