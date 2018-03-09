/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

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

// Adjustments to new Publication Timing and Execution Framework 
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using System;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class TwistReceiver : MessageReceiver
    {
        private float previousRealTime;
        private Vector3 linearVelocity;
        private Vector3 angularVelocity;
        private bool isMessageReceived;

        public override Type MessageType { get { return (typeof(GeometryTwist)); } }
        private GeometryTwist message;

        private void Awake()
        {
            MessageReception += ReceiveMessage;
        }

        private void ReceiveMessage(object sender, MessageEventArgs e)
        {
            message = (GeometryTwist)e.Message;
            linearVelocity = getVector3(message.linear).Ros2Unity();
            angularVelocity = -getVector3(message.angular).Ros2Unity();
            isMessageReceived = true;
        }

        private static Vector3 getVector3(GeometryVector3 geometryVector3)
        {
            return new Vector3(
                geometryVector3.x,
                geometryVector3.y,
                geometryVector3.z);
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }
        private void ProcessMessage()
        {
            float deltaTime = Time.realtimeSinceStartup-previousRealTime;
            transform.Translate(linearVelocity * deltaTime);
            transform.Rotate(Vector3.forward, angularVelocity.x * deltaTime);
            transform.Rotate(Vector3.up, angularVelocity.y * deltaTime);
            transform.Rotate(Vector3.left, angularVelocity.z * deltaTime);
            previousRealTime = Time.realtimeSinceStartup;
            isMessageReceived = false;
        }     
    }
}