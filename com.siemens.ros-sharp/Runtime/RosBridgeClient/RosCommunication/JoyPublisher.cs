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

- Instead of using JoyAxisReader, now JoyPublisher seeks for IAxisReader interfaces.
    © Siemens AG, 2024 Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com)
*/

using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public class JoyPublisher : UnityPublisher<MessageTypes.Sensor.Joy>
    {
        // private JoyAxisReader[] JoyAxisReaders;
        private List<IAxisReader> axisReaders = new List<IAxisReader>();
        private JoyButtonReader[] JoyButtonReaders;

        public string FrameId = "Unity";
        public float CycleTime = 0.1f;
        private MessageTypes.Sensor.Joy message;
        private float nextPublishTime;

        protected override void Start()
        {
            base.Start();
            InitializeGameObject();
            InitializeMessage();
        }

        private void Update()
        {
            if (Time.time >= nextPublishTime)
            {
                nextPublishTime = Time.time + CycleTime;
                UpdateMessage();
            }
        }      

        private void InitializeGameObject()
        {
            axisReaders = FindObjectsOfType<MonoBehaviour>().OfType<IAxisReader>().ToList();
            
            // JoyAxisReaders = GetComponents<JoyAxisReader>();
            JoyButtonReaders = GetComponents<JoyButtonReader>();            
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Sensor.Joy();
            message.header.frame_id = FrameId;

            // message.axes = new float[JoyAxisReaders.Length];
            message.axes = new float[axisReaders.Count];

            message.buttons = new int[JoyButtonReaders.Length];
        }

        private void UpdateMessage()
        {
            message.header.Update();

            // for (int i = 0; i < JoyAxisReaders.Length; i++)
            //     message.axes[i] = JoyAxisReaders[i].Read();

            for (int i = 0; i < axisReaders.Count; i++)
                message.axes[i] = axisReaders[i].Read();
            
            for (int i = 0; i < JoyButtonReaders.Length; i++)
                message.buttons[i] = (JoyButtonReaders[i].Read() ? 1 : 0);

            Publish(message);
        }  
    }
}
