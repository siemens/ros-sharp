/*
© Siemens AG, 2017
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

#if UNITY_EDITOR

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfInertial))]
    public class UrdfInertialEditor : UnityEditor.Editor
    {

        Rigidbody rigidbody;
        SerializedProperty pDisplayInertiaGizmo;
        SerializedProperty pRigidbodyDataSource;

        SerializedProperty pMass;
        SerializedProperty pCenterOfMass;
        SerializedProperty pInertiaTensor;
        SerializedProperty pInertiaTensorRotation;

        protected virtual void OnEnable()
        {
            rigidbody = (Rigidbody)serializedObject.FindProperty("_rigidbody").objectReferenceValue;
            pDisplayInertiaGizmo = serializedObject.FindProperty("DisplayInertiaGizmo");
            pRigidbodyDataSource = serializedObject.FindProperty("rigidbodyDataSource");
            pMass = serializedObject.FindProperty("Mass");
            pCenterOfMass = serializedObject.FindProperty("CenterOfMass");
            pInertiaTensor = serializedObject.FindProperty("InertiaTensor");
            pInertiaTensorRotation = serializedObject.FindProperty("InertiaTensorRotation");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            GUILayout.Space(5);
            EditorGUILayout.PropertyField(pDisplayInertiaGizmo, new GUIContent("Display Inertia Gizmo"));
            GUILayout.Space(5);

            EditorGUILayout.PropertyField(pRigidbodyDataSource, new GUIContent("Rigidbody Data Source"));

            bool isEnabled = (UrdfInertial.RigidbodyDataSource)pRigidbodyDataSource.enumValueIndex != UrdfInertial.RigidbodyDataSource.Manual;

            EditorGUI.BeginDisabledGroup(isEnabled);
            pMass.floatValue = EditorGUILayout.FloatField("Mass", rigidbody.mass);
            pCenterOfMass.vector3Value = EditorGUILayout.Vector3Field("Center of Mass", rigidbody.centerOfMass);
            pInertiaTensor.vector3Value = EditorGUILayout.Vector3Field("Inertia Tensor", rigidbody.inertiaTensor);
            Vector3 angles = EditorGUILayout.Vector3Field("Inertia Tensor Rotation", rigidbody.inertiaTensorRotation.eulerAngles);
            pInertiaTensorRotation.quaternionValue = Quaternion.Euler(angles);
            EditorGUI.EndDisabledGroup();

            serializedObject.ApplyModifiedProperties();
        }

    }
}

#endif