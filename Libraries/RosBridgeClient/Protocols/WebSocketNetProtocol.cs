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

// this class requires .NET 4.5+ to compile and Windows 8+ to work

using System;
using System.Linq;
using System.Net.WebSockets;
using System.Threading;

namespace RosSharp.RosBridgeClient.Protocols
{
    public class WebSocketNetProtocol : IProtocol
    {
        public event EventHandler OnReceive;

        private ClientWebSocket ClientWebSocket;
        private string Uri { get; }
        private const int bufferSize = 1024*10;

        public WebSocketNetProtocol(string uri)
        {
            ClientWebSocket = new ClientWebSocket();
            Uri = uri;
        }

        public async void Connect()
        {
            await ClientWebSocket.ConnectAsync(new Uri(Uri), CancellationToken.None);
            StartListen();
        }

        public void Close()
        {
            ClientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
        }

        public bool IsAlive()
        {
            return ClientWebSocket.State == WebSocketState.Open;
        }

        public void Send(byte[] data)
        {
            ClientWebSocket.SendAsync(new ArraySegment<byte>(data), WebSocketMessageType.Text, true, CancellationToken.None);
        }

        private async void StartListen()
        {
            while (ClientWebSocket.State == WebSocketState.Open)
            {
                var buffer = new byte[bufferSize];
                var message = new byte[0];

                WebSocketReceiveResult result;
                do
                {
                    result = await ClientWebSocket.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);
                    message = message.Concat(buffer).ToArray();
                } while (!result.EndOfMessage);

                if (result.MessageType == WebSocketMessageType.Close)
                    await ClientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
                else
                    OnReceive.Invoke(this, new MessageEventArgs(message));
            }
        }

    }
}
