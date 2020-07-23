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

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    
    [CustomEditor(typeof(HingeJointLimitsManager))]
    public class HingeJointLimitsManagerEditor : UnityEditor.Editor
    {
        private const float toleranceThreshold = 10;

        SerializedProperty pLargeAngleLimitMin;
        SerializedProperty pLargeAngleLimitMax;
        SerializedProperty pTolerance;
        SerializedProperty pAngleActual;
        SerializedProperty pRotationNumberActual;
        SerializedProperty pRotationNumberMin;
        SerializedProperty pRotationNumberMax;
        SerializedProperty pAngleLimitMin;
        SerializedProperty pAngleLimitMax;

        protected virtual void OnEnable()
        {
            pLargeAngleLimitMin = serializedObject.FindProperty("LargeAngleLimitMin");
            pLargeAngleLimitMax = serializedObject.FindProperty("LargeAngleLimitMax");
            pTolerance = serializedObject.FindProperty("Tolerance");
            pAngleActual = serializedObject.FindProperty("AngleActual");
            pRotationNumberActual = serializedObject.FindProperty("RotationNumberActual");
            pRotationNumberMin = serializedObject.FindProperty("RotationNumberMin");
            pRotationNumberMax = serializedObject.FindProperty("RotationNumberMax");
            pAngleLimitMin = serializedObject.FindProperty("AngleLimitMin");
            pAngleLimitMax = serializedObject.FindProperty("AngleLimitMax");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            pLargeAngleLimitMin.floatValue = EditorGUILayout.FloatField("Large Angle Limit Min: ", pLargeAngleLimitMin.floatValue);
            pLargeAngleLimitMax.floatValue = EditorGUILayout.FloatField("Large Angle Limit Max: ", pLargeAngleLimitMax.floatValue);
            pTolerance.floatValue = EditorGUILayout.FloatField("Tolerance: ", pTolerance.floatValue);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(new GUIContent("Actual Angle:"), new GUIContent(pAngleActual.floatValue.ToString("F2")));
            EditorGUILayout.LabelField(new GUIContent("Actual Rotation No.:"), new GUIContent(pRotationNumberActual.intValue.ToString()));
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(new GUIContent("Min Angle:"), new GUIContent(pAngleLimitMin.floatValue.ToString("F2")));
            EditorGUILayout.LabelField(new GUIContent("Min. No. of Rotations:"), new GUIContent(pRotationNumberMin.intValue.ToString()));
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField(new GUIContent("Max Angle:"), new GUIContent(pAngleLimitMax.floatValue.ToString("F2")));
            EditorGUILayout.LabelField(new GUIContent("Max. No. of Rotations:"), new GUIContent(pRotationNumberMax.intValue.ToString()));
            EditorGUILayout.EndHorizontal();

            if (180 - pAngleLimitMin.floatValue < toleranceThreshold)
                EditorGUILayout.HelpBox("Min. Angle is close to +180° where this fix will not work properly. Please increase tolerance.", MessageType.Warning);

            if (180 - pAngleLimitMax.floatValue < toleranceThreshold)
                EditorGUILayout.HelpBox("Max. Angle is close to -180° where this fix will not work properly. Please increase tolerance.", MessageType.Warning);

            serializedObject.ApplyModifiedProperties();
        }
    }
}