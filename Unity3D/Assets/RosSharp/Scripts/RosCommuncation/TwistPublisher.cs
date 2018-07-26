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
    public class TwistPublisher : Publisher<Messages.Geometry.Twist>
    {
        private Messages.Geometry.Twist message;

        private float previousRealTime;        
        private Vector3 previousPosition = Vector3.zero;
        private Quaternion previousRotation = Quaternion.identity;

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
            message = new Messages.Geometry.Twist();
            message.linear = new Messages.Geometry.Vector3();
            message.angular = new Messages.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            float deltaTime = Time.realtimeSinceStartup - previousRealTime;

            Vector3 linearVelocity = (transform.position - previousPosition)/deltaTime;
            Vector3 angularVelocity = (transform.rotation.eulerAngles - previousRotation.eulerAngles)/deltaTime;
                
            message.linear = GetGeometryVector3(linearVelocity.Unity2Ros()); ;
            message.angular = GetGeometryVector3(- angularVelocity.Unity2Ros());

            previousRealTime = Time.realtimeSinceStartup;
            previousPosition = transform.position;
            previousRotation = transform.rotation;

            Publish(message);
        }

        private static Messages.Geometry.Vector3 GetGeometryVector3(Vector3 vector3)
        {
            Messages.Geometry.Vector3 geometryVector3 = new Messages.Geometry.Vector3();
            geometryVector3.x = vector3.x;
            geometryVector3.y = vector3.y;
            geometryVector3.z = vector3.z;
            return geometryVector3;
        }
    }
}
