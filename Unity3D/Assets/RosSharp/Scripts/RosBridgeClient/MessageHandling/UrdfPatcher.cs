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
using System.Collections.Generic;
using RosSharp.Urdf;

namespace RosSharp.RosBridgeClient
{

    public class UrdfPatcher : MonoBehaviour
    {
        public GameObject UrdfModel;

        public bool EnableRigidbodiesGravity;
        public bool SetRigidbodiesKinematic;
        public bool SetMeshCollidersConvex;

        public bool AddJointStateReaders;
        public bool AddJointStateWriters;

        public void Patch()
        {
            RemoveExistingComponents();

            PatchRigidbodies(EnableRigidbodiesGravity, SetRigidbodiesKinematic);
            PatchMeshColliders(SetMeshCollidersConvex);

            if (AddJointStateReaders)
            {                
                JointStatePublisher jointStatePublisher = AddComponentIfNotExists<JointStatePublisher>();                
                jointStatePublisher.JointStateReaders = AddJointStateReaderComponents();
            }

            if (AddJointStateWriters)
            {
                JointStateSubscriber jointStateSubscriber = AddComponentIfNotExists<JointStateSubscriber>();
                AddJointStateWriterComponents(out jointStateSubscriber.JointNames, out jointStateSubscriber.JointStateWriters);
            }
        }
        
        private JointStateReader[] AddJointStateReaderComponents() 
        {
            List<JointStateReader> jointStateReaders = new List<JointStateReader>();
            foreach (UrdfJoint urdfJoint in UrdfModel.GetComponentsInChildren<UrdfJoint>())
                jointStateReaders.Add(urdfJoint.gameObject.AddComponent<JointStateReader>());   
            return jointStateReaders.ToArray();
        }

        private void AddJointStateWriterComponents(out List<string> jointNames, out List<JointStateWriter> jointStateWriters)
        {
            jointNames = new List<string>();
            jointStateWriters = new List<JointStateWriter>();

            foreach (UrdfJoint urdfJoint in UrdfModel.GetComponentsInChildren<UrdfJoint>()) {
                jointNames.Add(urdfJoint.JointName);
                jointStateWriters.Add(urdfJoint.gameObject.AddComponent<JointStateWriter>());
            }
        }

        private void RemoveExistingComponents()
        {
            foreach (Transform child in UrdfModel.GetComponentsInChildren<Transform>())
            {
                child.DestroyImmediateIfExists<JointStateReader>();
                child.DestroyImmediateIfExists<JointStateWriter>();
            }
        }

        private void PatchMeshColliders(bool convex)
        {
            foreach (MeshCollider meshCollider in UrdfModel.GetComponentsInChildren<MeshCollider>())
            {
                meshCollider.convex = convex;
            }
        }

        private void PatchRigidbodies(bool useGravity,bool isKinematic)
        {
            foreach (Rigidbody rigidbody in UrdfModel.GetComponentsInChildren<Rigidbody>())
            {
                rigidbody.useGravity = useGravity;
                rigidbody.isKinematic = isKinematic;
            }
        }

        private T AddComponentIfNotExists<T>() where T : Component
        {
            T component = GetComponent<T>();
            if (component == null)
                component = gameObject.AddComponent<T>();
            return component;
        }

    }
}
