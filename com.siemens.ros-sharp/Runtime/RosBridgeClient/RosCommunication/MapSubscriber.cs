/*
© Siemens AG, 2024
Author: hoyadong1 (github.com/hoyadong1)

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
    public class MapSubscriber : UnitySubscriber<MessageTypes.Nav.OccupancyGrid>
    {
        public sbyte[] mapData;
        public int width;
        public int height;
        public float resolution;
        public Vector3 originPosition;

        public bool isMapReceived = false;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Nav.OccupancyGrid message)
        {
            width = (int)message.info.width;
            height = (int)message.info.height;
            resolution = message.info.resolution;
            originPosition = new Vector3(
                (float)message.info.origin.position.x,
                (float)message.info.origin.position.y,
                (float)message.info.origin.position.z
            );

            mapData = message.data;
            isMapReceived = true;
        }
    }
}