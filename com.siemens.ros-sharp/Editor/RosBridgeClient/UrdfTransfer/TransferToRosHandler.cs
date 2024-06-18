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

* Removed additional RosConnector instance. Urdf transfer is now handled by the existing RosConnector component.
* If no RosConnector component is present in the scene, one can be created by pressing the button.
* RosConnector specific input fields have been removed as they are no longer required.
* Robot name parameter input field added.
* The 'Reset to Default' button now behaves according to the selected ROS version (from the RosConnector component). 
* Added GUI hints for parameter syntax. 
    (C) Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

using System.Threading;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.IO;
using RosSharp.RosBridgeClient.UrdfTransfer;

namespace RosSharp.RosBridgeClient
{
    /// <summary>
    /// Handles the transfer of URDF files to ROS.
    /// </summary>
    public class TransferToRosHandler
    {
        public RosSocket RosSocket;
        private RosConnector rosConnector;

        public Dictionary<string, ManualResetEvent> StatusEvents;

        /// <summary>
        /// Initializes a new instance of the <see cref="TransferToRosHandler"/> class.
        /// </summary>
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

        /// <summary>
        /// Transfers the URDF file to ROS.
        /// </summary>
        /// <param name="urdfPath">The path to the URDF file.</param>
        /// <param name="rosPackage">The name of the ROS package.</param>
        /// <param name="robotNameParameter">The parameter name for the robot name.</param>
        public void Transfer(string urdfPath, string rosPackage, string robotNameParameter)
        {
            RosSocket = RosConnector.ConnectToRos(rosConnector.protocol, rosConnector.RosBridgeServerUrl, OnConnected, OnClose, rosConnector.Serializer);

            if (!StatusEvents["connected"].WaitOne(rosConnector.SecondsTimeout))
            {
                Debug.LogError("RosSocket cannot connect to server. Is the file server running?");
                RosSocket.Close();
                return;
            }

            Debug.Log("Connected to " + rosConnector.RosBridgeServerUrl);

            if (Path.GetExtension(urdfPath)?.ToLowerInvariant() != ".urdf")
            {
                Debug.LogWarning("Please select a valid URDF file to publish. Closing connection.");
                RosSocket.Close();
                return;
            }

            Thread transferToRos = new Thread(() => TransferAsync(urdfPath, rosPackage, robotNameParameter));
            transferToRos.Start();
        }

        private void TransferAsync(string urdfPath, string rosPackage, string robotNameParameter)
        {
            if (!StatusEvents["connected"].WaitOne(rosConnector.SecondsTimeout * 1000))
            {
                Debug.LogWarning("Failed to connect to " + rosConnector.RosBridgeServerUrl + " before timeout.");
                RosSocket.Close();
                return;
            }

            string robotName = Path.GetFileName(urdfPath);

            UrdfTransferToRos urdfTransferToRos = new UrdfTransferToRos(RosSocket, robotName, robotNameParameter, urdfPath, rosPackage);

            StatusEvents["robotNamePublished"] = urdfTransferToRos.Status["robotNamePublished"];
            StatusEvents["robotDescriptionPublished"] = urdfTransferToRos.Status["robotDescriptionPublished"];
            StatusEvents["resourceFilesSent"] = urdfTransferToRos.Status["resourceFilesSent"];

            urdfTransferToRos.Transfer();
        }

        /// <summary>
        /// Checks if a RosConnector component exists in the scene.
        /// </summary>
        /// <returns><c>true</c> if a RosConnector component exists; otherwise, <c>false</c>.</returns>
        public bool CheckForRosConnector() {
            rosConnector = GameObject.FindObjectOfType(typeof(RosConnector)) as RosConnector;
            return rosConnector != null;
        }

        /// <summary>
        /// Creates a new RosConnector component in the scene.
        /// </summary>
        public void CreateRosConnector()
        {
            GameObject newGameObject = new GameObject("RosConnectorObject");
            rosConnector = newGameObject.AddComponent<RosConnector>();
            RosConnectorEditor.IsInitialized = false;
            RosConnectorEditor.ToggleROSVersion(rosConnector.selectedROSVersion);
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
