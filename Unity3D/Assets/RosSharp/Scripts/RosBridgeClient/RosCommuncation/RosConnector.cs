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
using System.Threading.Tasks;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        public int Timeout = 3;

        public RosSocket RosSocket { get; private set; }
        public enum Protocols { WebSocketSharp, WebSocketNET };
        public RosBridgeClient.RosSocket.SerializerEnum Serializer;
        public Protocols Protocol;
        public string RosBridgeServerUrl = "ws://192.168.0.1:9090";

        private bool isConnected = false;

        public void Awake()
        {
            RosSocket = ConnectToRos(Protocol, RosBridgeServerUrl, OnConnected, OnClosed, Serializer);
            Connect();
        }

        private void Connect()
        {
            RosSocket.protocol.Connect();
        }

        public static RosSocket ConnectToRos(Protocols protocolType, string serverUrl, EventHandler onConnected = null, EventHandler onClosed = null, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON)
        {
            RosBridgeClient.Protocols.IProtocol protocol = GetProtocol(protocolType, serverUrl);
            protocol.OnConnected += onConnected;
            protocol.OnClosed += onClosed;

            return new RosSocket(protocol, serializer);
        }

        private static RosBridgeClient.Protocols.IProtocol GetProtocol(Protocols protocol, string rosBridgeServerUrl)
        {
            switch (protocol)
            {
                case Protocols.WebSocketSharp:
                    return new RosBridgeClient.Protocols.WebSocketSharpProtocol(rosBridgeServerUrl);
                case Protocols.WebSocketNET:
                    return new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);
                default:
                    return null;
            }
        }

        private void OnApplicationQuit()
        {
            RosSocket.Close();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            isConnected = true;
            Debug.Log("Connected to RosBridge: " + RosBridgeServerUrl);
        }

        private async void OnClosed(object sender, EventArgs e)
        {
            isConnected = false;
            Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl + ". Retrying in " + Timeout + " seconds.");
            await Task.Delay(Timeout * 1000);
            Connect();
        }
    }
}
