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
using System.Net.WebSockets;
using System.Threading;

namespace RosSharp.RosBridgeClient.Protocols
{


    public class WebSocketNetProtocol : IProtocol
    {
        public event EventHandler OnReceive;

        private ClientWebSocket ClientWebSocket;
        private string Uri { get; }
        private const int maxFrameSize = (int)2e8;
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
                int bufferSize = 1000;
                var buffer = new byte[bufferSize];
                var offset = 0;
                var free = buffer.Length;
                WebSocketReceiveResult result;
                do
                {
                    result = await ClientWebSocket.ReceiveAsync(new ArraySegment<byte>(buffer, offset, free), CancellationToken.None);
                    offset += result.Count;
                    free -= result.Count;
                    if (free == 0)
                    {
                        // No free space
                        // Resize the outgoing buffer
                        var newSize = buffer.Length + bufferSize;
                        // Check if the new size exceeds a limit
                        // It should suit the data it receives
                        // This limit however has a max value of 2 billion bytes (2 GB)
                        if (newSize > maxFrameSize)
                        {
                            throw new Exception("Maximum size exceeded");
                        }
                        var newBuffer = new byte[newSize];
                        Array.Copy(buffer, 0, newBuffer, 0, offset);
                        buffer = newBuffer;
                        free = buffer.Length - offset;
                    }
                } while (!result.EndOfMessage);
                if (result.MessageType == WebSocketMessageType.Close)
                    await ClientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
                else
                    OnReceive.Invoke(this, new MessageEventArgs(buffer));
            }
        }

    }
}
