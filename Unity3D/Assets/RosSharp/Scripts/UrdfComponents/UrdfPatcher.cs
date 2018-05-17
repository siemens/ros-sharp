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
using System.Text.RegularExpressions;

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
        public JointStateProvider jointStateProvider;
        public bool AddJointStateWriters;
        public JointStateReceiver jointStateReceiver;

        public struct JointInfo
        {
            public JointStateHandler.JointTypes jointType;
            public string jointName;
        };

        Dictionary<Transform, JointInfo> jointTypeDictionary;

        public void Patch()
        {
            RemoveExistingComponents();

            PatchRigidbodies(EnableRigidbodiesGravity, SetRigidbodiesKinematic);

            PatchMeshColliders(SetMeshCollidersConvex);

            if (AddPoseProvider)
                UrdfModel.AddComponent<PoseProvider>();

            if (AddPoseReceiver)
                UrdfModel.AddComponent<PoseReceiver>();

            if (AddJointStateReaders || AddJointStateWriters)
                GetSingleDimensionalJoints();

            if (AddJointStateReaders)
                jointStateProvider.JointStateReaders = PatchJoints<JointStateReader>();

            if (AddJointStateWriters)
                jointStateReceiver.JointStateWriters = PatchJoints<JointStateWriter>();
        }

        private void GetSingleDimensionalJoints()
        {
            jointTypeDictionary =  new Dictionary<Transform, JointInfo>();

            JointInfo jointInfo;

            foreach (Transform child in UrdfModel.GetComponentsInChildren<Transform>())            
                if (HasSingleDimensionalJoint(child, out jointInfo))
                    jointTypeDictionary.Add(child, jointInfo);
        
        }

        public T[] PatchJoints<T>() where T : JointStateHandler
        {
            int jointID = 0;
            T[] jointStateHandlers = new T[jointTypeDictionary.Count];
            
            foreach (KeyValuePair<Transform, JointInfo> jointInfoEntry in jointTypeDictionary)
            {
                jointStateHandlers[jointID] = jointInfoEntry.Key.gameObject.AddComponent<T>();
                jointStateHandlers[jointID].JointType = jointInfoEntry.Value.jointType;
                jointStateHandlers[jointID].JointName = jointInfoEntry.Value.jointName;
                jointStateHandlers[jointID].JointID = jointID++;
                
            }
            return jointStateHandlers;
        }


        private bool HasSingleDimensionalJoint(Transform child, out JointInfo jointInfo)
        {
            jointInfo.jointType = JointStateHandler.JointTypes.continuous;
            jointInfo.jointName = null;

            string linkName = null;
            string jointTypeName = null;
            Match match = Regex.Match(child.name, @"^([^(\s]+)\s+[(]([^):]+):\s+([^)\s]+)[)]$");
            if (match.Success)
            {
                linkName = match.Groups[1].Value;
                jointTypeName = match.Groups[2].Value;
                jointInfo.jointName = match.Groups[3].Value;
            }

            if (child.name.Contains("continuous Joint"))
                jointInfo.jointType = JointStateHandler.JointTypes.continuous;
            else if (child.name.Contains("revolute Joint"))
                jointInfo.jointType = JointStateHandler.JointTypes.revolute;
            else if (child.name.Contains("prismatic Joint"))
                jointInfo.jointType = JointStateHandler.JointTypes.prismatic;
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
