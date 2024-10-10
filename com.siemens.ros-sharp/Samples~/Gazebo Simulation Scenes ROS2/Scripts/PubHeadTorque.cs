/*
© Siemens AG, 2024
Author: Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com)

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
using RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEngine;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class Float64Publisher : UnityPublisher<MessageTypes.Std.Float64>
    {
        private Double DoubleData = 0;

        private MessageTypes.Std.Float64 message;

        public Slider slider;


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
            message = new MessageTypes.Std.Float64();
            message.data = DoubleData;
        }
        public void UpdateMessage()
        {
            message.data = -slider.value;
            Publish(message);
        }

    }
}
