/*
© Siemens AG, 2017-2019
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
using System.Threading;

namespace RosSharp.RosBridgeClient.Protocols
{
    public class RosConnector
    {
        public int timeout;

        public RosSocket rosSocket { get; private set; }
        public RosSocket.SerializerEnum serializer;
        public Protocol protocol;
        public string rosBridgeServerUrl;

        private ManualResetEvent isConnected = new ManualResetEvent(false);

        public RosConnector(string url, Protocol protocol, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, int timeout = 10)
        {
            this.rosBridgeServerUrl = url;
            this.protocol = protocol;
            this.serializer = serializer;
            this.timeout = timeout;
        }

        public bool ConnectAndWait()
        {
            rosSocket = ConnectToRos(protocol, rosBridgeServerUrl, OnConnected, OnClosed, serializer);
            if (!isConnected.WaitOne(timeout * 1000)) {
                Console.WriteLine("Failed to connect to RosBridge at: " + rosBridgeServerUrl);
                return false;
            }
            return true;
        }

        public static RosSocket ConnectToRos(Protocol protocolType, string serverUrl, EventHandler onConnected = null, EventHandler onClosed = null, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON)
        {
            IProtocol protocol = GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;

            return new RosSocket(protocol, serializer);
        }

        private static IProtocol GetProtocol(Protocol protocol, string rosBridgeServerUrl)
        {
            switch (protocol)
            {
                case Protocol.WebSocketSharp:
                    return new RosBridgeClient.Protocols.WebSocketSharpProtocol(rosBridgeServerUrl);
                case Protocol.WebSocketNET:
                    return new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);
                default:
                    return null;
            }
        }

        private void OnApplicationQuit()
        {
            rosSocket.Close();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            isConnected.Set();
        }

        private void OnClosed(object sender, EventArgs e)
        {
            isConnected.Reset();
        }
    }

    public enum Protocol { WebSocketSharp, WebSocketNET };
}