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

namespace RosSharp.Urdf.Editor
{
    
    [CustomEditor(typeof(HingeJointLimitsManager))]
    public class HingeJointLimitsManagerEditor : UnityEditor.Editor
    {
        private const float toleranceThreshold = 10;

        private HingeJointLimitsManager hingeJointLimitsManager;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            hingeJointLimitsManager = (HingeJointLimitsManager)target;
            //if (EditorGUILayout.Foldout(true, "Angles"))

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Actual Angle:", hingeJointLimitsManager.AngleActual.ToString());
            EditorGUILayout.LabelField("Actual Rotation No.:", hingeJointLimitsManager.RotationNumberActual.ToString());
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Min Angle:", hingeJointLimitsManager.AngleLimitMin.ToString());
            EditorGUILayout.LabelField("Min. No. of Rotations:", hingeJointLimitsManager.RotationNumberMin.ToString());
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Max Angle:", hingeJointLimitsManager.AngleLimitMax.ToString());
            EditorGUILayout.LabelField("Max. No. of Rotations:", hingeJointLimitsManager.RotationNumberMax.ToString());
            EditorGUILayout.EndHorizontal();

            if (180 - hingeJointLimitsManager.AngleLimitMin < toleranceThreshold)
                EditorGUILayout.HelpBox("Min. Angle is close to +180° where this fix will not work properly. Please increase tolerance.", MessageType.Warning);

            if (180 - hingeJointLimitsManager.AngleLimitMax < toleranceThreshold)
                EditorGUILayout.HelpBox("Max. Angle is close to -180° where this fix will not work properly. Please increase tolerance.", MessageType.Warning);

        }
    }
}