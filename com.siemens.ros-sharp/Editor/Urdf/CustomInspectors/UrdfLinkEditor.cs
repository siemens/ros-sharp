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

#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfLink))]
    public class UrdfLinkEditor : UnityEditor.Editor
    {
        private UrdfLink urdfLink;
        private UrdfJoint.JointTypes jointType = UrdfJoint.JointTypes.Fixed;

        protected virtual void OnEnable()
        {
            urdfLink = (UrdfLink)serializedObject.targetObject;
        }

        public override void OnInspectorGUI()
        {
            GUILayout.Space(5);
            urdfLink.IsBaseLink = EditorGUILayout.Toggle("Is Base Link", urdfLink.IsBaseLink);
            GUILayout.Space(5);

            EditorGUILayout.BeginVertical("HelpBox");
            jointType = (UrdfJoint.JointTypes) EditorGUILayout.EnumPopup(
                "Child Joint Type", jointType);

            if (GUILayout.Button("Add child link (with joint)"))
            {
                UrdfLink childLink = UrdfLinkExtensions.Create(urdfLink.transform);
                UrdfJoint.Create(childLink.gameObject, jointType);
            }
            EditorGUILayout.EndVertical();
        }
    }
}

#endif