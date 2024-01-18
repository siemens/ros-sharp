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

namespace RosSharp.RosBridgeClient
{
    public class OdometrySubscriber : UnitySubscriber<MessageTypes.Nav.Odometry>
    {
        public Transform PublishedTransform;

        private Vector3 position;
        private Quaternion rotation;
        private bool isMessageReceived;

        protected override void Start()
		{
			base.Start();
		}
		
        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Nav.Odometry message)
        {
            position = GetPosition(message).Ros2Unity();
            rotation = GetRotation(message).Ros2Unity();
            isMessageReceived = true;
        }
        private void ProcessMessage()
        {
            PublishedTransform.position = position;
            PublishedTransform.rotation = rotation;
        }

        private Vector3 GetPosition(MessageTypes.Nav.Odometry message)
        {
            return new Vector3(
                (float)message.pose.pose.position.x,
                (float)message.pose.pose.position.y,
                (float)message.pose.pose.position.z);
        }

        private Quaternion GetRotation(MessageTypes.Nav.Odometry message)
        {
            return new Quaternion(
                (float)message.pose.pose.orientation.x,
                (float)message.pose.pose.orientation.y,
                (float)message.pose.pose.orientation.z,
                (float)message.pose.pose.orientation.w);
        }
    }
}