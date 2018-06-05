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
    public class PoseProvider : MessageProvider
    {
        private Messages.Geometry.PoseStamped message;
        public override Type MessageType { get { return (typeof(Messages.Geometry.PoseStamped)); } }

        public string FrameId;

        private void Start()
        {
            InitializeMessage();
        }
        private void FixedUpdate()
        {
            if (IsMessageRequested)
                UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new Messages.Geometry.PoseStamped();
            message.header = new Messages.Standard.Header();
            message.header.frame_id = FrameId;
        }
        private void UpdateMessage()
        {
            message.header.Update();
            message.pose.position = GetGeometryPoint(transform.position.Unity2Ros());
            message.pose.orientation = GetGeometryQuaternion(transform.rotation.Unity2Ros());
            RaiseMessageRelease(new MessageEventArgs(message));
        }

        private Messages.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            Messages.Geometry.Point geometryPoint = new Messages.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }
        private Messages.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            Messages.Geometry.Quaternion geometryQuaternion = new Messages.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }
    }
}
