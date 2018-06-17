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

namespace RosSharp.RosBridgeClient
{

    public class UrdfPatcher : MonoBehaviour
    {
        public GameObject UrdfModel;
        
        public bool EnableRigidbodiesGravity;
        public bool SetRigidbodiesKinematic;
        public bool SetMeshCollidersConvex;

        public bool AddPoseProvider;
        public bool AddPoseReceiver;
        public bool AddJointStateReaders;
        public JointStatePublisher jointStateProvider;
        public bool AddJointStateWriters;
        public JointStateSubscriber jointStateReceiver;

        public void Patch()
        {
            RemoveExistingComponents();

            PatchRigidbodies(EnableRigidbodiesGravity, SetRigidbodiesKinematic);

            PatchMeshColliders(SetMeshCollidersConvex);

            if (AddPoseProvider)
                UrdfModel.AddComponent<PoseStampedSubscriber>();

            if (AddPoseReceiver)
                UrdfModel.AddComponent<OdometrySubscriber>();

            if (AddJointStateReaders)
                jointStateProvider.JointStateReaders = AddJointStateReaderComponents();

            if (AddJointStateWriters)
                jointStateReceiver.JointStateWriterDictionary = AddJointStateWriterComponents();
        }
        
        private JointStateReader[] AddJointStateReaderComponents() 
        {
            List<JointStateReader> jointStateReaders = new List<JointStateReader>();
            foreach (JointUrdfDataManager jointUrdfDataManager in UrdfModel.GetComponentsInChildren<JointUrdfDataManager>())
                jointStateReaders.Add(jointUrdfDataManager.gameObject.AddComponent<JointStateReader>());   
            return jointStateReaders.ToArray();
        }

        private Dictionary<string, JointStateWriter> AddJointStateWriterComponents()
        {
            Dictionary<string, JointStateWriter> jointStateWriters = new Dictionary<string, JointStateWriter>();
            foreach (JointUrdfDataManager jointUrdfDataManager in UrdfModel.GetComponentsInChildren<JointUrdfDataManager>())
                jointStateWriters.Add(
                    jointUrdfDataManager.name,
                    jointUrdfDataManager.gameObject.AddComponent<JointStateWriter>());
            return jointStateWriters;
        }

        private void RemoveExistingComponents()
        {
            foreach (Transform child in UrdfModel.GetComponentsInChildren<Transform>())
            {
                child.DestroyImmediateIfExists<JointStateReader>();
                child.DestroyImmediateIfExists<JointStateWriter>();
                child.DestroyImmediateIfExists<PoseStampedSubscriber>();
                child.DestroyImmediateIfExists<OdometrySubscriber>();
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

    }
}
