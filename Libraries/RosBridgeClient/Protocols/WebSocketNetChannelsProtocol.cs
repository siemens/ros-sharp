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
using System.Threading.Channels;
using System.Threading.Tasks;

namespace RosSharp.RosBridgeClient.Protocols
{
    public class WebSocketNetChannelsProtocol : IProtocol
    {
        private ClientWebSocket clientWebSocket;
        private readonly Uri uri;
        private readonly CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();
        private readonly CancellationTokenSource protocolTokenSource = new CancellationTokenSource();
        private readonly CancellationToken cancellationToken;
        private ManualResetEvent IsConnected = new ManualResetEvent(false);
        private AutoResetEvent IsReadyToSend = new AutoResetEvent(true);

        private const int ReceiveChunkSize = 1024;
        private const int SendChunkSize = 1024;

        private ChannelReader<ArraySegment<byte>> reader;
        private ChannelWriter<ArraySegment<byte>> writer;

        private Task listener;
        private Task sender;

        public event EventHandler OnReceive;
        public event EventHandler OnConnected;
        public event EventHandler OnClosed;
        public event EventHandler OnSent;

        public WebSocketNetChannelsProtocol(string uriString, int queueSize = 1000)
        {
            Channel<ArraySegment<byte>> channel = Channel.CreateUnbounded<ArraySegment<byte>>(new UnboundedChannelOptions()
            {
                AllowSynchronousContinuations = false,
                SingleReader = true,
                SingleWriter = false,
            });

            reader = channel.Reader;
            writer = channel.Writer;

            clientWebSocket = new ClientWebSocket();
            uri = new Uri(uriString);
            cancellationToken = cancellationTokenSource.Token;
        }

        public void Connect()
        {
            Task.Run(() => ConnectAsync());
        }

        public async void ConnectAsync()
        {
            await clientWebSocket.ConnectAsync(uri, cancellationToken);
            IsConnected.Set();
            OnConnected?.Invoke(null, EventArgs.Empty);

            listener = Task.Run(StartListen);
            sender = Task.Run(StartSend);
        }

        public void Close()
        {
            if (IsAlive())
            {
                // No more new messages will get accepted
                writer.Complete();
            }
        }

        public bool IsAlive()
        {
            return clientWebSocket.State == WebSocketState.Open;
        }

        public void Send(byte[] message)
        {
            Send(new ArraySegment<byte>(message));
        }

        public void Send(ArraySegment<byte> message)
        {
            if (!writer.TryWrite(message))
            {
                throw new InternalBufferOverflowException("Rosbridge channel is full");
            }
        }

        private async Task StartSend()
        {
            while (await reader.WaitToReadAsync())
            {
                if (reader.TryRead(out ArraySegment<byte> message))
                {
                    if (clientWebSocket.State != WebSocketState.Open)
                        throw new WebSocketException(WebSocketError.InvalidState, "Error Sending Message. WebSocket State is: " + clientWebSocket.State);

                    int messageCount = (int)Math.Ceiling((double)message.Count / SendChunkSize);

                    for (int i = 0; i < messageCount; i++)
                    {
                        int offset = SendChunkSize * i;
                        bool endOfMessage = (i == messageCount - 1);
                        int count = endOfMessage ? message.Count - offset : SendChunkSize;
                        await clientWebSocket.SendAsync(new ArraySegment<byte>(message.Array, offset, count), WebSocketMessageType.Binary, endOfMessage, cancellationToken).ConfigureAwait(false);
                    }

                    // Example, return the array to an array pool
                    OnSent?.Invoke(this, new MessageEventArgs(message.Array));
                }

            }

            // close the socket (listener will therminate after that)
            clientWebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None).Wait();

            IsConnected.Reset();
            OnClosed?.Invoke(null, EventArgs.Empty);
        }

        private async Task StartListen()
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