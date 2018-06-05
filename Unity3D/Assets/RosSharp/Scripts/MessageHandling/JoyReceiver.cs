﻿/*
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
using System;

namespace RosSharp.RosBridgeClient
{

    public class JoyReceiver : MessageReceiver
    {
        public override Type MessageType { get { return (typeof(Messages.Sensor.Joy)); } }
        private Messages.Sensor.Joy message;

        public JoyButtonWriter[] joyButtonWriters;
        public JoyAxisWriter[] joyAxisWriters;

        private void Awake()
        {
            MessageReception += ReceiveMessage;
        }

        private void ReceiveMessage(object sender, MessageEventArgs e)
        {            
            message = (Messages.Sensor.Joy)e.Message;

            int I = joyButtonWriters.Length < message.buttons.Length ? joyButtonWriters.Length : message.buttons.Length;
            for (int i = 0; i < I; i++)
                if (joyButtonWriters[i]!=null)
                joyButtonWriters[i].Write(message.buttons[i]);

            I = joyAxisWriters.Length < message.axes.Length ? joyAxisWriters.Length : message.axes.Length;
            for (int i = 0; i < I; i++)
                if (joyAxisWriters[i] != null)
                    joyAxisWriters[i].Write(message.axes[i]);
        }
    }
}
