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
     public class PoseReceiver : MessageReceiver
    {
        public enum PoseMessageTypes { GeometryPoseStamped, NavigationOdometry };
        public PoseMessageTypes PoseMessageType;

        private Type type;

        private Vector3 position;
        private Quaternion rotation;
        private  bool isMessageReceived;

        public override Type MessageType { get { return type; } }

        private void Awake()
        {
            SetType();
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }
        private void SetType()
        {
            if (PoseMessageType == PoseMessageTypes.GeometryPoseStamped)
            {
                type = typeof(GeometryPoseStamped);
                MessageReception += ReceiveGeometryPoseStampedMessage;
            }
            else if (PoseMessageType == PoseMessageTypes.NavigationOdometry)
            {
                type = typeof(NavigationOdometry);
                MessageReception += ReceiveNaviagtionOdometryMessage;
            }
        }

        private void ReceiveGeometryPoseStampedMessage(object sender, MessageEventArgs e)
        {
            position = GetPosition((GeometryPoseStamped)e.Message).Ros2Unity();
            rotation = GetRotation((GeometryPoseStamped)e.Message).Ros2Unity();
            isMessageReceived = true;
        }
        private void ReceiveNaviagtionOdometryMessage(object sender, MessageEventArgs e)
        {
            position = GetPosition((NavigationOdometry)e.Message).Ros2Unity();
            rotation = GetRotation((NavigationOdometry)e.Message).Ros2Unity();
            isMessageReceived = true;
        }
        private void ProcessMessage()
        {
            transform.position = position;
            transform.rotation = rotation;
        }        

        private Vector3 GetPosition(GeometryPoseStamped message)
        {
            return new Vector3(
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z);
        }
        private Vector3 GetPosition(NavigationOdometry message)
        {
            return new Vector3(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z);
        }

        private Quaternion GetRotation(GeometryPoseStamped message)
        {
            return new Quaternion(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w);
        }

        private Quaternion GetRotation(NavigationOdometry message)
        {
            return new Quaternion(
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w);
        }
    }
}