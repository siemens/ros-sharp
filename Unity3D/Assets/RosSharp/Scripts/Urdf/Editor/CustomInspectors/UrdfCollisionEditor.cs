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

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfCollision))]
    class UrdfCollisionEditor : UnityEditor.Editor
    {
        SerializedProperty pGeometryType;
        private Transform transform;

        protected virtual void OnEnable()
        {
            pGeometryType = serializedObject.FindProperty("GeometryType");
            transform = ((UrdfCollision)target).transform;
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            GUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel("Geometry Type");
            EditorGUILayout.LabelField(((GeometryTypes) pGeometryType.enumValueIndex).ToString());
            EditorGUILayout.EndHorizontal();

            DisplayWarnings();

            serializedObject.ApplyModifiedProperties();
        }

        private void DisplayWarnings()
        {
            if (!transform.HasExactlyOneChild())
            {
                GUILayout.Space(5);
                EditorGUILayout.HelpBox("Visual element must have one and only one child Geometry element.", MessageType.Error);
            }
            else if (UrdfGeometry.IsTransformed(transform.GetChild(0), (GeometryTypes)pGeometryType.enumValueIndex))
            {
                GUILayout.BeginVertical("HelpBox");
                EditorGUILayout.HelpBox("Changes to the transform of the child Geometry element cannot be exported to URDF. " +
                                        "Make any translation, rotation, or scale changes to this Visual object instead.", MessageType.Error);

                if (GUILayout.Button("Fix transformations"))
                {
                    //Only transfer rotation if geometry is not a mesh
                    bool transferRotation = (GeometryTypes)pGeometryType.enumValueIndex != GeometryTypes.Mesh;
                    transform.MoveChildTransformToParent(transferRotation);
                }
                GUILayout.EndVertical();
            }
        }
    }
}
