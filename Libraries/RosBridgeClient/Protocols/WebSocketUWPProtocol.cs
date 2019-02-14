/*© David Whitney, 2018
Author: David Whitney (david_whitney@brown.edu)

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

#if WINDOWS_UWP

using Windows.Networking.Sockets;
using Windows.Foundation;
using Windows.Storage.Streams;
using System;


namespace RosSharp.RosBridgeClient.Protocols
{
    public class WebSocketUWPProtocol : IProtocol
    {

        public event EventHandler OnReceive;
        public event EventHandler OnConnected;
        public event EventHandler OnClosed;

        MessageWebSocket WebSocket;
        Uri Url;
        DataWriter MessageWriter;


        public WebSocketUWPProtocol(string Url)
        {
            this.Url = TryGetUri(Url);
            WebSocket = new MessageWebSocket();
            WebSocket.Control.MaxMessageSize = uint.MaxValue;
            WebSocket.MessageReceived += WebSocket_MessageReceived;
            WebSocket.Closed += WebSocket_Closed;
        }

        private void WebSocket_Closed(IWebSocket sender, WebSocketClosedEventArgs args)
        {
            OnClosed?.Invoke(null, EventArgs.Empty);
        }

        public bool isAlive = false;

        public bool IsAlive()
        {
            return isAlive;
        }

        public void Connect()
        {
            WebSocket.ConnectAsync(this.Url).Completed = (source, status) =>
                {
                    if (status == AsyncStatus.Completed)
                    {
                        MessageWriter = new DataWriter(WebSocket.OutputStream);
                        isAlive = true;
                        OnConnected?.Invoke(null, EventArgs.Empty);
                    }
                };
        }

        public void Close()
        {
            if (WebSocket != null)
            {
                WebSocket.Dispose();
                WebSocket = null;
                isAlive = false;
                OnClosed?.Invoke(null, EventArgs.Empty);
            } 
        }

        public async void Send(byte[] data)
        {
            if (WebSocket != null && MessageWriter != null)
            {
                try
                {
                    MessageWriter.WriteBytes(data);
                    await MessageWriter.StoreAsync();
                }
                catch
                {
                    return;
                }
            } 
        }

        private void WebSocket_MessageReceived(MessageWebSocket sender, MessageWebSocketMessageReceivedEventArgs args)
        {
            if (args.IsMessageComplete)
            {
                try
                {
                    using (var reader = args.GetDataReader())
                    {
                        var messageLength = args.GetDataReader().UnconsumedBufferLength;
                        byte[] receivedMessage = new byte[messageLength];
                        reader.UnicodeEncoding = UnicodeEncoding.Utf8;
                        reader.ReadBytes(receivedMessage);
                        OnReceive?.Invoke(this, new MessageEventArgs(receivedMessage));

                    }
                }
                catch
                {
                    return;
                }
            }
        } 

        static System.Uri TryGetUri(string uriString)
        {
            Uri webSocketUri;
            if (!Uri.TryCreate(uriString.Trim(), UriKind.Absolute, out webSocketUri))
                throw new System.Exception("Error: Invalid URI");

            // Fragments are not allowed in WebSocket URIs.
            if (!String.IsNullOrEmpty(webSocketUri.Fragment))
                throw new System.Exception("Error: URI fragments not supported in WebSocket URIs.");

            // Uri.SchemeName returns the canonicalized scheme name so we can use case-sensitive, ordinal string
            // comparison.
            if ((webSocketUri.Scheme != "ws") && (webSocketUri.Scheme != "wss"))
                throw new System.Exception("Error: WebSockets only support ws:// and wss:// schemes.");

            return webSocketUri;
        }
    }
};

#endif