﻿/*
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
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class TransferFromRosHandler
    {
        private string robotName;
        private string localDirectory;

        private int timeout;
        private string assetPath;
        private string urdfParameter;
        private string robotNameParameter; 
        private string urdfFileName;

        private RosSocket rosSocket;
        public RosConnector rosConnector;

        public Dictionary<string, ManualResetEvent> StatusEvents;

        public TransferFromRosHandler()
        {
            StatusEvents = new Dictionary<string, ManualResetEvent>{
                { "connected", new ManualResetEvent(false) },
                { "robotNameReceived",new ManualResetEvent(false) },
                { "robotDescriptionReceived", new ManualResetEvent(false) },
                { "resourceFilesReceived", new ManualResetEvent(false) },
                { "disconnected", new ManualResetEvent(false) },
                { "importComplete", new ManualResetEvent(false) }
                };
        }

        public void TransferUrdf(string assetPath, string urdfParameter, string robotNameParameter)
        {
            timeout = rosConnector.SecondsTimeout;
            this.assetPath = assetPath;
            this.urdfParameter = urdfParameter;
            this.robotNameParameter = robotNameParameter;

            // Initialize
            ResetStatusEvents();

            // Awake RosConnector
            rosSocket = RosConnector.ConnectToRos(rosConnector.protocol, rosConnector.RosBridgeServerUrl, OnConnected, OnClosed, rosConnector.Serializer);

            if (!StatusEvents["connected"].WaitOne(timeout * 1000))
            {
                Debug.LogWarning("Failed to connect to ROS before timeout");
                return;
            }

            ImportAssets();
        }

        private void ImportAssets()
        {
            // setup Urdf Transfer
            UrdfTransferFromRos urdfTransfer = new UrdfTransferFromRos(rosSocket, assetPath, urdfParameter, robotNameParameter);
            StatusEvents["robotNameReceived"] = urdfTransfer.Status["robotNameReceived"];
            StatusEvents["robotDescriptionReceived"] = urdfTransfer.Status["robotDescriptionReceived"];
            StatusEvents["resourceFilesReceived"] = urdfTransfer.Status["resourceFilesReceived"];

            urdfTransfer.Transfer();
            urdfFileName = urdfTransfer.UrdfFileName;

            if (StatusEvents["robotNameReceived"].WaitOne(timeout * 1000))
            {
                robotName = urdfTransfer.RobotName;
                localDirectory = urdfTransfer.LocalUrdfDirectory;
            }

            // import URDF assets:
            if (StatusEvents["resourceFilesReceived"].WaitOne(timeout * 1000))
                Debug.Log("Imported urdf resources to " + localDirectory);
            else
                Debug.LogWarning("Not all resource files have been received before timeout.");

            rosSocket.Close();
        }

        public void GenerateModelIfReady()
        {
            if (!StatusEvents["resourceFilesReceived"].WaitOne(0) || StatusEvents["importComplete"].WaitOne(0))
                return;

            AssetDatabase.Refresh();

            if (EditorUtility.DisplayDialog(
                "Urdf Assets imported.",
                "Do you want to generate a " + robotName + " GameObject now?",
                "Yes", "No"))
            {
                Urdf.Editor.UrdfRobotExtensions.Create(Path.Combine(localDirectory, urdfFileName));
            }

            StatusEvents["importComplete"].Set();
        }

        public bool CheckForRosConnector() {
            rosConnector = GameObject.FindObjectOfType(typeof(RosConnector)) as RosConnector;
            return rosConnector != null;    
        }

        public void CreateRosConnector()
        {
            GameObject newGameObject = new GameObject("RosConnectorObject");
            rosConnector = newGameObject.AddComponent<RosConnector>();
        }

        private void OnClosed(object sender, EventArgs e)
        {
            StatusEvents["disconnected"].Set();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            StatusEvents["connected"].Set();
        }

        private void ResetStatusEvents()
        {
            foreach (var manualResetEvent in StatusEvents.Values)
                manualResetEvent.Reset();
        }
    }
}
