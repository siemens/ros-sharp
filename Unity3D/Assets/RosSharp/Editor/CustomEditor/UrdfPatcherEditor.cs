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
    [CustomEditor(typeof(UrdfPatcher))]
    public class UrdfPatcherEditor : Editor
    {
        private UrdfPatcher urdfPatcher;

        public override void OnInspectorGUI()
        {
            urdfPatcher = (UrdfPatcher)target;

            //DrawDefaultInspector();
            
            urdfPatcher.UrdfModel = (GameObject)EditorGUILayout.ObjectField("Urdf Model", urdfPatcher.UrdfModel, typeof(GameObject), true);
            GUILayout.Space(10);
            urdfPatcher.EnableRigidbodiesGravity = GUILayout.Toggle(urdfPatcher.EnableRigidbodiesGravity, "Enable Gravity for Rigidbodies");
            urdfPatcher.SetRigidbodiesKinematic = GUILayout.Toggle(urdfPatcher.SetRigidbodiesKinematic, "Set Rigidbodies Kinematic");
            urdfPatcher.SetMeshCollidersConvex = GUILayout.Toggle(urdfPatcher.SetMeshCollidersConvex, "Set Mesh Colliders Convex");

            //GUILayout.Space(10);
            //urdfPatcher.AddJointStateReaders = GUILayout.Toggle(urdfPatcher.AddJointStateReaders, "Publish Joint States (Add Joint State Readers)");
            //if (urdfPatcher.AddJointStateReaders)
            //    urdfPatcher.jointStatePublisher = (JointStatePublisher) EditorGUILayout.ObjectField("Joint State Provider", urdfPatcher.jointStatePublisher, typeof(JointStatePublisher), true);

            //urdfPatcher.AddJointStateWriters = GUILayout.Toggle(urdfPatcher.AddJointStateWriters, "Subscribe Joint States (Add Joint State Writers)");
            //if (urdfPatcher.AddJointStateWriters)
            //    urdfPatcher.jointStateSubscriber = (JointStateSubscriber) EditorGUILayout.ObjectField("Joint State Receiver", urdfPatcher.jointStateSubscriber, typeof(JointStateSubscriber), true);

            GUILayout.Space(10);
            if (GUILayout.Button("Apply"))
                urdfPatcher.Patch();
        }
    }
}