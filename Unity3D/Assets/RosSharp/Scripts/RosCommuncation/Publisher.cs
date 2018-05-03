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
    [RequireComponent(typeof(RosConnector))]
    public abstract class Publisher : MonoBehaviour
    {
        public string Topic;
        public MessageProvider MessageProvider;

        public event EventHandler PublicationEvent;

        protected RosSocket rosSocket;
        protected string publicationId;

        protected virtual void Start()
        {
            rosSocket = GetComponent<RosConnector>().RosSocket;

            publicationId = rosSocket.Advertise(Topic, MessageTypes.RosMessageType(MessageProvider.MessageType));
            PublicationEvent += ReadMessage;
        }

        protected void ReadMessage(object sender, EventArgs e)
        {
            MessageProvider.RaiseMessageRequest(e);
            MessageProvider.MessageRealease += Publish;
        }

        protected void Publish(object sender, MessageEventArgs e)
        {
            MessageProvider.MessageRealease -= Publish;
            rosSocket.Publish(publicationId, e.Message);
        }
        protected virtual void StartPublication(EventArgs e)
        {
            EventHandler eventHandler = PublicationEvent;
            if (eventHandler != null)
                eventHandler(this, e);
        }
    }
}