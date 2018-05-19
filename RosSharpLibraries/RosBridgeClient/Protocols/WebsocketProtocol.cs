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
using WebSocketSharp;

namespace RosSharp.RosBridgeClient.Protocols
{
    public class WebsocketProtocol: IProtocol
    {
        public event EventHandler OnMessage;

        private WebSocket WebSocket;

        public WebsocketProtocol(string url)
        {
            WebSocket = new WebSocket(url);
            WebSocket.OnMessage += OnWebsocketMessage;                
        }
                
        public void Connect()
        {
            WebSocket.Connect();            
        }

        public void Close()
        {
            WebSocket.Close();
        }

        public bool IsAlive()
        {
            return WebSocket.IsAlive;
        }

        public void SendAsync(byte[] data, Action<bool> completed)
        {
            WebSocket.SendAsync(data, completed);
        }
        
        private void OnWebsocketMessage(object sender, WebSocketSharp.MessageEventArgs e)
        {            
            OnMessage.Invoke(sender, new MessageEventArgs(e.RawData));
        }
    }
}
