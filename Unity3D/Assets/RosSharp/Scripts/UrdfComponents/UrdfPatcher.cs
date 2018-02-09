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
using System;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{

    public class UrdfPatcher : MonoBehaviour
    {

        public GameObject UrdfModel;
        public bool SetMeshCollidersConvex;
        public bool EnableRigidbodiesGravity;
        public bool SetRigidbodiesKinematic;
        public bool AddJointStateReaders;
        public JointStateProvider jointStateProvider;
        public bool AddJointStateWriters;
        public JointStateReceiver jointStateReceiver;
        public bool AddPoseReceiver;
        public bool AddPoseProvider;

        Dictionary<Transform, JointStateHandler.JointTypes> jointTypeDictionary;

        public void Patch()
        {
            RemoveExistingComponents();
            PatchMeshColliders(SetMeshCollidersConvex);
            PatchRigidbodies(EnableRigidbodiesGravity, SetRigidbodiesKinematic);

            if (AddJointStateReaders || AddJointStateWriters)
                GetSingleDimensionalJoints();

            if (AddJointStateReaders)
                jointStateProvider.JointStateReaders = PatchJoints<JointStateReader>();

            if (AddJointStateWriters)
                jointStateReceiver.JointStateWriters = PatchJoints<JointStateWriter>();

            if (AddPoseReceiver)
                UrdfModel.AddComponent<PoseReceiver>();

            if (AddPoseProvider)
                UrdfModel.AddComponent<PoseProvider>();

        }

        private void GetSingleDimensionalJoints()
        {
            jointTypeDictionary =  new Dictionary<Transform, JointStateHandler.JointTypes>();

            JointStateHandler.JointTypes jointType;

            foreach (Transform child in UrdfModel.GetComponentsInChildren<Transform>())            
                if (HasSingleDimensionalJoint(child, out jointType))
                    jointTypeDictionary.Add(child, jointType);
        
        }

        public T[] PatchJoints<T>() where T : JointStateHandler
        {
            int jointID = 0;
            T[] jointStateHandlers = new T[jointTypeDictionary.Count];
            
            foreach (KeyValuePair<Transform,JointStateHandler.JointTypes> jointTypeEntry in jointTypeDictionary)
            {
                jointStateHandlers[jointID] = jointTypeEntry.Key.gameObject.AddComponent<T>();
                jointStateHandlers[jointID].JointType = jointTypeEntry.Value;
                jointStateHandlers[jointID].JointID = jointID++;
                
            }
            return jointStateHandlers;
        }


        private bool HasSingleDimensionalJoint(Transform child, out JointStateReader.JointTypes jointType)
        {
            jointType = JointStateHandler.JointTypes.continuous;

            if (child.name.Contains("continuous Joint"))
                jointType = JointStateHandler.JointTypes.continuous;
            else if (child.name.Contains("revolute Joint"))
                jointType = JointStateHandler.JointTypes.revolute;
            else if (child.name.Contains("prismatic Joint"))
                jointType = JointStateHandler.JointTypes.prismatic;
            else
                return false;

            return true;
        }

        private void RemoveExistingComponents()
        {
            foreach (Transform child in UrdfModel.GetComponentsInChildren<Transform>())
            {
                child.DestroyImmediateIfExists<JointStateReader>();
                child.DestroyImmediateIfExists<JointStateWriter>();
                child.DestroyImmediateIfExists<PoseReceiver>();
                child.DestroyImmediateIfExists<PoseProvider>();
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
