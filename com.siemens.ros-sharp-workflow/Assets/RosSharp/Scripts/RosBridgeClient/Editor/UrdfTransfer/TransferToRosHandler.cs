/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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

using System.Threading;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.IO;
using RosSharp.RosBridgeClient.UrdfTransfer;

namespace RosSharp.RosBridgeClient
{
    public class TransferToRosHandler
    {
        public RosSocket RosSocket;

        public Dictionary<string, ManualResetEvent> StatusEvents;

        public TransferToRosHandler()
        {
            StatusEvents = new Dictionary<string, ManualResetEvent>
            {
                {"connected", new ManualResetEvent(false) },
                {"robotNamePublished", new ManualResetEvent(false)},
                {"robotDescriptionPublished", new ManualResetEvent(false)},
                {"resourceFilesSent", new ManualResetEvent(false) }
            };
        }

        public void Transfer(Protocols.Protocol protocolType, string serverUrl, int timeout, string urdfPath, string rosPackage, RosSocket.SerializerEnum serializer)
        {
            if (Path.GetExtension(urdfPath)?.ToLowerInvariant() != ".urdf")
            {
                Debug.LogWarning("Please select a valid URDF file to publish.");
                return;
            }
                        
            Thread transferToRos = new Thread(() => TransferAsync(protocolType, serverUrl, timeout, urdfPath, rosPackage, serializer));
            transferToRos.Start();
        }

        private void TransferAsync(Protocols.Protocol protocolType, string serverUrl, int timeout, string urdfPath, string rosPackage, RosSocket.SerializerEnum serializer)
        {
            RosSocket = RosConnector.ConnectToRos(protocolType, serverUrl, OnConnected, OnClose, serializer);

            if (!StatusEvents["connected"].WaitOne(timeout * 1000))
            {
                Debug.LogWarning("Failed to connect to " + serverUrl + " before timeout.");
                RosSocket.Close();
                return;
            }

            string robotName = Path.GetFileName(urdfPath);
            UrdfTransferToRos urdfTransferToRos = new UrdfTransferToRos(RosSocket, robotName, urdfPath, rosPackage);

            StatusEvents["robotNamePublished"] = urdfTransferToRos.Status["robotNamePublished"];
            StatusEvents["robotDescriptionPublished"] = urdfTransferToRos.Status["robotDescriptionPublished"];
            StatusEvents["resourceFilesSent"] = urdfTransferToRos.Status["resourceFilesSent"];

            urdfTransferToRos.Transfer();
        }

        private void OnClose(object sender, EventArgs e)
        {
            StatusEvents["connected"].Reset();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            StatusEvents["connected"].Set();
        }
    }
}
