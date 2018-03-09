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

using System;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class JointStateReceiver : MessageReceiver
    {
        public override Type MessageType { get { return (typeof(SensorJointStates)); } }

        public JointStateWriter[] JointStateWriters;

        private SensorJointStates message;

        private void Awake()
        {
            MessageReception += ReceiveMessage;
        }

        private void ReceiveMessage(object sender, MessageEventArgs e)
        {
            message = (SensorJointStates)e.Message;
            for (int i = 0; i < JointStateWriters.Length; i++)
                JointStateWriters[i].Write(message.position[i]);   
        }
    }
}

