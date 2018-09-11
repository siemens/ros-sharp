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

using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class JointStatePublisher : Publisher<Messages.Sensor.JointState>
    {
        private Messages.Sensor.JointState message;        
        public string FrameId = "Unity";

        public List<JointStateReader> JointStateReaders;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            int jointStateLength = JointStateReaders.Count;
            message = new Messages.Sensor.JointState
            {
                header = new Messages.Standard.Header { frame_id = FrameId },
                name = new string[jointStateLength],
                position = new float[jointStateLength],
                velocity = new float[jointStateLength],
                effort = new float[jointStateLength]
            };
        }
        private void UpdateMessage()
        {
            message.header.Update();
            for (int i = 0; i < JointStateReaders.Count; i++)
                UpdateJointState(i);

            Publish(message);
        }

        private void UpdateJointState(int i)
        {
            JointStateReaders[i].Read(
                out message.name[i],
                out message.position[i],
                out message.velocity[i],
                out message.effort[i]);
        }
    }
}
