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
    public class WebSocketSharpProtocol: IProtocol
    {
        public event EventHandler OnReceive;
        public event EventHandler OnSent;
        public event EventHandler OnConnected;
        public event EventHandler OnClosed;

        private WebSocket WebSocket;

        public WebSocketSharpProtocol(string url)
        {
            WebSocket = new WebSocket(url);
            WebSocket.OnMessage += Receive;

            WebSocket.OnClose += Closed;
            WebSocket.OnOpen += Connected;
        }
                
        public void Connect()
        {
            WebSocket.ConnectAsync();            
        }

        public void Close()
        {
            WebSocket.CloseAsync();
        }

        public bool IsAlive()
        {
            return WebSocket.IsAlive;
        }

        public void Send(byte[] data)
        {
            WebSocket.SendAsync(data, null);
            OnSent?.Invoke(this, new MessageEventArgs(data));
        }
        
        public void Send(ArraySegment<byte> data)
        {
            throw new NotImplementedException();
        }

        private void Receive(object sender, WebSocketSharp.MessageEventArgs e)
        {
            OnReceive?.Invoke(sender, new MessageEventArgs(e.RawData));
        }

        private void Closed(object sender, EventArgs e)
        {
            OnClosed?.Invoke(sender, e);
        }

        private void Connected(object sender, EventArgs e)
        {
            OnConnected?.Invoke(sender, e);
        }
    }
}
