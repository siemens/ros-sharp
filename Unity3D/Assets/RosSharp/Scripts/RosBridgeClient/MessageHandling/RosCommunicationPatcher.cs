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
    public class RosCommunicationPatcher : MonoBehaviour
    {
        public UrdfRobot urdfRobot;

        public void SetPublishJointStates(bool publish) 
        {
            if (publish)
            {
                JointStatePublisher jointStatePublisher = transform.AddComponentIfNotExists<JointStatePublisher>();
                jointStatePublisher.JointStateReaders = new List<JointStateReader>();

                foreach (UrdfJoint urdfJoint in urdfRobot.GetComponentsInChildren<UrdfJoint>())
                    jointStatePublisher.JointStateReaders.Add(urdfJoint.transform.AddComponentIfNotExists<JointStateReader>());
            }
            else
            {
                GetComponent<JointStatePublisher>()?.JointStateReaders.Clear();

                foreach (JointStateReader reader in urdfRobot.GetComponentsInChildren<JointStateReader>())
                    reader.transform.DestroyImmediateIfExists<JointStateReader>();
            }
        }

        public void SetSubscribeJointStates(bool subscribe)
        {
            if (subscribe)
            {
                JointStateSubscriber jointStateSubscriber = transform.AddComponentIfNotExists<JointStateSubscriber>();
                jointStateSubscriber.JointStateWriters = new List<JointStateWriter>();
                jointStateSubscriber.JointNames = new List<string>();

                foreach (UrdfJoint urdfJoint in urdfRobot.GetComponentsInChildren<UrdfJoint>()) {
                    jointStateSubscriber.JointStateWriters.Add(urdfJoint.transform.AddComponentIfNotExists<JointStateWriter>());
                    jointStateSubscriber.JointNames.Add(urdfJoint.JointName);
                }
            }
            else
            {
                GetComponent<JointStateSubscriber>()?.JointStateWriters.Clear();
                GetComponent<JointStateSubscriber>()?.JointNames.Clear();

                foreach (JointStateWriter writer in urdfRobot.GetComponentsInChildren<JointStateWriter>())
                    writer.transform.DestroyImmediateIfExists<JointStateWriter>();
            }
        }
    }
}
