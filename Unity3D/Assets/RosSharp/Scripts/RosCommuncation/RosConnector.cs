/*
© Siemens AG, 2017
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

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosConnector : MonoBehaviour
    {
        public int timeout = 10;

        public RosSocket RosSocket { get; private set; }
        public enum Protocols { WebSocketSharp, WebSocketNET };
        public Protocols Protocol;
        public string RosBridgeServerUrl = "ws://192.168.0.1:9090";

        public void Awake()
        {
            RosSocket = new RosSocket(GetProtocol());
            if (RosSocket.Protocol.WaitForConnection(timeout))
                Debug.Log("Connected to RosBridge: " + RosBridgeServerUrl);
            else
                Debug.LogWarning("Failed to connect to RosBridge at: " + RosBridgeServerUrl);
        }

        private RosBridgeClient.Protocols.Protocol GetProtocol()
        {
            switch (Protocol)
            {
                case Protocols.WebSocketSharp:
                    return new RosBridgeClient.Protocols.WebSocketSharpProtocol(RosBridgeServerUrl);
                default:
                    return new RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl);
            }
        }

        public void Disconnect()
        {
            RosSocket.Close();
            if (RosSocket.Protocol.WaitForDisconnection(timeout))
                Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl);
            else
                Debug.Log("Failed to disconnect from RosBridge before timeout: " + RosBridgeServerUrl);
        }

        private void OnApplicationQuit()
        {
            RosSocket.Close();
            if(RosSocket.Protocol.WaitForDisconnection(timeout))
                Debug.Log("Disconnected from RosBridge: " + RosBridgeServerUrl);
            else
                Debug.Log("Failed to disconnect from RosBridge before timeout: " + RosBridgeServerUrl);
        }
    }
}