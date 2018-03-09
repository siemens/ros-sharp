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

namespace RosSharp.RosBridgeClient
{
    public class JoyProvider : MessageProvider
    {
        private SensorJoy message;
        public override Type MessageType { get { return (typeof(SensorJoy)); } }
  
        public string FrameId;     

        private JoyAxisReader[] JoyAxisReaders;
        private JoyButtonReader[] JoyButtonReaders;
        
        private void Start()
        {
            InitializeGameObject();
            InitializeMessage();
        }

        private void Update()
        {
            if (IsMessageRequested)
                UpdateMessage();
        }      
        private void InitializeGameObject()
        {
            JoyAxisReaders = GetComponents<JoyAxisReader>();
            JoyButtonReaders = GetComponents<JoyButtonReader>();            
        }
        private void InitializeMessage()
        {
            message = new SensorJoy();
            message.header.frame_id = FrameId;
            message.axes = new float[JoyAxisReaders.Length];
            message.buttons = new int[JoyButtonReaders.Length];
        }
        private void UpdateMessage()
        {
            message.header.Update();

            for (int i = 0; i < JoyAxisReaders.Length; i++)
                message.axes[i] = JoyAxisReaders[i].Read();
            
            for (int i = 0; i < JoyButtonReaders.Length; i++)
                message.buttons[i] = (JoyButtonReaders[i].Read() ? 1 : 0);

            RaiseMessageRelease(new MessageEventArgs(message));
        }
    }
}
