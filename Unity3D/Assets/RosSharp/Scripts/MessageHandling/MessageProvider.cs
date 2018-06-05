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
    public abstract class MessageProvider : MonoBehaviour
    {       
        public abstract Type MessageType { get; }
        public event EventHandler MessageRequest;
        public event EventHandler<MessageEventArgs> MessageRealease;

        protected bool IsMessageRequested { get { return isMessageRequested; } }
        private bool isMessageRequested;

        private void Awake()
        {
            MessageRequest += OnMessageRequest;
            MessageRealease += OnMessageRelease;
        }
        private void OnMessageRelease(object sender, MessageEventArgs e)
        {
            isMessageRequested = false;
        }
        private void OnMessageRequest(object sender, EventArgs e)
        {
            isMessageRequested = true;
        }
        
        protected virtual void RaiseMessageRelease(MessageEventArgs e)
        {
            MessageRealease?.Invoke(this, e);
        }

        public virtual void RaiseMessageRequest(EventArgs e)
        {
            MessageRequest?.Invoke(this, e);
        }
    }
}