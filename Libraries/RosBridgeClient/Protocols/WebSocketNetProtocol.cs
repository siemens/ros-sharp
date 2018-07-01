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

// this class (System.Net.WebSockets) requires .NET 4.5+ to compile and Windows 8+ to work

using System;
using System.IO;
using System.Net.WebSockets;
using System.Threading;

namespace RosSharp.RosBridgeClient.Protocols
{
    public class WebSocketNetProtocol : IProtocol
    {
        private ClientWebSocket clientWebSocket;
        private readonly Uri uri;
        private readonly CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();
        private readonly CancellationToken cancellationToken;
        private AutoResetEvent SendAsyncReady = new AutoResetEvent(true);
        private AutoResetEvent ReceiveAsyncReady = new AutoResetEvent(true);

        private const int ReceiveChunkSize = 1024;
        private const int SendChunkSize = 1024;

        public event EventHandler OnReceive;

        public WebSocketNetProtocol(string uriString)
        {
            clientWebSocket = new ClientWebSocket();
            uri = new Uri(uriString);
            cancellationToken = cancellationTokenSource.Token;
        }

        public async void Connect()
        {
            await clientWebSocket.ConnectAsync(uri, cancellationToken);
            StartListen();
        }

        public async void Close()
        {
            await clientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
        }

        public bool IsAlive()
        {
            return clientWebSocket.State == WebSocketState.Open;
        }

        public void Send(byte[] message)
        {
            SendAsync(message);
        }

        public async void SendAsync(byte[] message)
        {
            int messageCount = (int)Math.Ceiling((double)message.Length / SendChunkSize);

            for (int i = 0; i < messageCount; i++)
            {
                int offset = SendChunkSize * i;
                bool endOfMessage = (i == messageCount - 1);
                int count = endOfMessage ? message.Length - offset : SendChunkSize;
                SendAsyncReady.WaitOne();
                await clientWebSocket.SendAsync(new ArraySegment<byte>(message, offset, count), WebSocketMessageType.Text, endOfMessage, cancellationToken);
                SendAsyncReady.Set();
            }
        }

        private async void StartListen()
        {
            byte[] buffer = new byte[ReceiveChunkSize];

            while (clientWebSocket.State == WebSocketState.Open)
            {
                MemoryStream memoryStream = new MemoryStream();
                WebSocketReceiveResult result;
                do
                {
                    ReceiveAsyncReady.WaitOne();
                    result = await clientWebSocket.ReceiveAsync(new ArraySegment<byte>(buffer), cancellationToken);
                    ReceiveAsyncReady.Set();

                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        await clientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
                        return;
                    }
                    else
                    {
                        memoryStream.Write(buffer, 0, result.Count);                        
                    }

                } while (!result.EndOfMessage);
                OnReceive.Invoke(this, new MessageEventArgs(memoryStream.ToArray()));
            }
        }
    }

}

