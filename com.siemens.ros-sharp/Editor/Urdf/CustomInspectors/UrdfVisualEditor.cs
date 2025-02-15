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
    [CustomEditor(typeof(UrdfVisual))]
    class UrdfVisualEditor : UnityEditor.Editor
    {
        private UrdfVisual urdfVisual;

        protected virtual void OnEnable()
        {
            urdfVisual = (UrdfVisual) serializedObject.targetObject;
        }

        public override void OnInspectorGUI()
        {
            GUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel("Geometry Type");
            EditorGUILayout.LabelField(urdfVisual.GeometryType.ToString());
            EditorGUILayout.EndHorizontal();
            
            if (GUILayout.Button("Add collision to match visual"))
            {
                urdfVisual.AddCorrespondingCollision();
            }

            DisplayWarnings();
        }

        private void DisplayWarnings()
        {
            if (!urdfVisual.transform.HasExactlyOneChild())
            {
                GUILayout.Space(5);
                EditorGUILayout.HelpBox("Visual element must have one and only one child Geometry element.", MessageType.Error);
            }
            else if (UrdfGeometry.IsTransformed(urdfVisual.transform.GetChild(0), urdfVisual.GeometryType))
            {
                GUILayout.Space(5);
                EditorGUILayout.HelpBox("Changes to the transform of the child Geometry element cannot be exported to URDF. " +
                                        "Make any translation, rotation, or scale changes to this Visual object instead.", MessageType.Error);

                if (GUILayout.Button("Fix transformations"))
                {
                    bool transferRotation = urdfVisual.GeometryType != GeometryTypes.Mesh;
                    urdfVisual.transform.MoveChildTransformToParent(transferRotation);
                }
            }
        }
    }
}

#endif