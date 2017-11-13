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

using UnityEditor;

namespace RosSharp.UrdfImporter
{
    [CustomEditor(typeof(RigidbodyData))]
    public class RigidbodyDataEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            RigidbodyData rigidbodyData = (RigidbodyData)target;

            bool newValue = EditorGUILayout.Toggle("Use URDF Data", rigidbodyData.UseUrdfData);
            EditorGUILayout.Vector3Field("URDF Center of Mass", rigidbodyData.CenterOfMass);
            EditorGUILayout.Vector3Field("URDF Inertia Tensor", rigidbodyData.InertiaTensor);
            EditorGUILayout.Vector3Field("URDF Inertia Tensor Rotation", rigidbodyData.InertiaTensorRotation.eulerAngles);
            if (newValue != rigidbodyData.UseUrdfData)
            {
                rigidbodyData.UseUrdfData = newValue;                    
                rigidbodyData.UpdateRigidBodyData();
            }
        }
    }
}