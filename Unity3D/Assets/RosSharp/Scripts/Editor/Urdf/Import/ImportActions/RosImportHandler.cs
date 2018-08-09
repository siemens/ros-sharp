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
using RosSharp.RosBridgeClient;
using System;
using UnityEngine;
using System.IO;
using UnityEditor;

namespace RosSharp.Urdf.Import
{
    class RosImportHandler
    {
        private string robotName;
        private string localDirectory;

        private int protocolNumber;
        private string address;
        private int timeout;
        private string assetPath;

        private RosSocket rosSocket;

        public Dictionary<string, ManualResetEvent> StatusEvents;

        public RosImportHandler()
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

        public void BeginRosImport(int protocolNumber, string address, int timeout, string assetPath)
        {
            this.protocolNumber = protocolNumber;
            this.address = address;
            this.timeout = timeout;
            this.assetPath = assetPath;

            // initialize
            ResetStatusEvents();

            var protocol = GetProtocol();
            protocol.OnConnected += OnConnected;
            protocol.OnClosed += OnClose;

            rosSocket = new RosSocket(protocol);
            ImportAssets();
        }

        private void ImportAssets()
        {
            if(!StatusEvents["connected"].WaitOne(timeout * 1000))
            {
                Debug.LogWarning("Failed to connect to ROS before timeout");
                return;
            }

            // setup urdfImporter
            RosBridgeClient.UrdfImporter urdfImporter = new RosBridgeClient.UrdfImporter(rosSocket, assetPath);
            StatusEvents["robotNameReceived"] = urdfImporter.Status["robotNameReceived"];
            StatusEvents["robotDescriptionReceived"] = urdfImporter.Status["robotDescriptionReceived"];
            StatusEvents["resourceFilesReceived"] = urdfImporter.Status["resourceFilesReceived"];

            urdfImporter.Import();

            if (StatusEvents["robotNameReceived"].WaitOne(timeout * 1000))
            {
                robotName = urdfImporter.RobotName;
                localDirectory = urdfImporter.LocalDirectory;
            }

            // import URDF assets:
            if (StatusEvents["resourceFilesReceived"].WaitOne(timeout * 1000))
                Debug.Log("Imported urdf resources to " + urdfImporter.LocalDirectory);
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
                RobotFactory.Create(Path.Combine(localDirectory, "robot_description.urdf"));
            }

            StatusEvents["importComplete"].Set();
        }

        private RosBridgeClient.Protocols.IProtocol GetProtocol()
        {
            switch (protocolNumber)
            {
                case 0: return new RosBridgeClient.Protocols.WebSocketSharpProtocol(address);
                default: return new RosBridgeClient.Protocols.WebSocketNetProtocol(address);
            }
        }

        private void OnClose(object sender, EventArgs e)
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
