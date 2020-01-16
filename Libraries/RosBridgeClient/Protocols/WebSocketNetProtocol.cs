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
        private ManualResetEvent IsConnected = new ManualResetEvent(false);
        private AutoResetEvent IsReadyToSend = new AutoResetEvent(true);

        private const int ReceiveChunkSize = 1024;
        private const int SendChunkSize = 1024;

        public event EventHandler OnReceive;
        public event EventHandler OnConnected;
        public event EventHandler OnClosed;
        public event EventHandler OnSent;

        public WebSocketNetProtocol(string uriString)
        {
            clientWebSocket = new ClientWebSocket();
            uri = new Uri(uriString);
            cancellationToken = cancellationTokenSource.Token;
        }

        public void Connect()
        {
            Thread thread = new Thread(() => ConnectAsync());
            thread.Start();
        }

        public async void ConnectAsync()
        {
            await clientWebSocket.ConnectAsync(uri, cancellationToken);
            IsConnected.Set();
            OnConnected?.Invoke(null, EventArgs.Empty);
            StartListen();
        }

        public async void Close()
        {
            if (IsAlive())
            {
                await clientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
                IsConnected.Reset();
                OnClosed?.Invoke(null, EventArgs.Empty);
            }
        }

        public bool IsAlive()
        {
            return clientWebSocket.State == WebSocketState.Open;
        }

        public void Send(byte[] message)
        {
            Thread thread = new Thread(() => SendAsync(message));
            thread.Start();
        }

        public void Send(ArraySegment<byte> data)
        {
            throw new NotImplementedException();
        }

        public async void SendAsync(byte[] message)
        {
            IsConnected.WaitOne();

            if (clientWebSocket.State != WebSocketState.Open)
                throw new WebSocketException(WebSocketError.InvalidState, "Error Sending Message. WebSocket State is: " + clientWebSocket.State);

            int messageCount = (int)Math.Ceiling((double)message.Length / SendChunkSize);

            IsReadyToSend.WaitOne();
            for (int i = 0; i < messageCount; i++)
            {
                int offset = SendChunkSize * i;
                bool endOfMessage = (i == messageCount - 1);
                int count = endOfMessage ? message.Length - offset : SendChunkSize;
                await clientWebSocket.SendAsync(new ArraySegment<byte>(message, offset, count), WebSocketMessageType.Binary, endOfMessage, cancellationToken);
            }

            IsReadyToSend.Set();
            OnSent?.Invoke(this, new MessageEventArgs(message));
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
                    result = await clientWebSocket.ReceiveAsync(new ArraySegment<byte>(buffer), cancellationToken);

                    if (result.MessageType == WebSocketMessageType.Close)
                        return;

                    memoryStream.Write(buffer, 0, result.Count);

                } while (!result.EndOfMessage);

                OnReceive?.Invoke(this, new MessageEventArgs(memoryStream.ToArray()));
            }
        }
    }
}

