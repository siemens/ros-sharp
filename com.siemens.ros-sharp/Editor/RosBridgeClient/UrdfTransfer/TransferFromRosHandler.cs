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

* Track resource file import progress (from UrdfTransferFromRos) to display a progress bar in the editor window
* Show final dialog only after the editor window triggers a repaint, instead of constantly checking for the import
complete status in the update loop.
* Final dialog for incomplete or failed resource file import, with a message to check the console for details.
* Configurable retry, retry-delay, and service-timeout settings to the transfer workflow.
* Cancellation support for active transfers when the editor window is closed or the ROS connection is lost.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
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
        private int maxRetries;
        private int retryDelayMs;
        private int serviceTimeoutMs;


        private RosSocket rosSocket;
        public RosConnector rosConnector;

        private UrdfTransferFromRos urdfTransfer;
        public float ResourceFileImportProgress => urdfTransfer?.ResourceFileImportProgress ?? 0f;

        private bool dialogAlreadyScheduled;

        private CancellationTokenSource cancellationTokenSource;

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

        public void TransferUrdf(
            string assetPath,
            string urdfParameter,
            string robotNameParameter,
            int maxRetries,
            int retryDelayMs,
            int serviceTimeoutMs)
        {
            timeout = rosConnector.SecondsTimeout;
            this.assetPath = assetPath;
            this.urdfParameter = urdfParameter;
            this.robotNameParameter = robotNameParameter;
            this.maxRetries = maxRetries;
            this.retryDelayMs = retryDelayMs;
            this.serviceTimeoutMs = serviceTimeoutMs;

            // Initialize
            ResetStatusEvents();

            // Awake RosConnector
            rosSocket = RosConnector.ConnectToRos(rosConnector.protocol, rosConnector.RosBridgeServerUrl, OnConnected, OnClosed, rosConnector.Serializer);

            // TODO: not the best practice here
            // "OnClosed" already sets the "disconnected" event
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
            urdfTransfer = new UrdfTransferFromRos(
                rosSocket,
                assetPath,
                urdfParameter,
                robotNameParameter,
                new Log(x => Debug.Log(x)),
                cancellationTokenSource.Token,
                maxRetries,
                retryDelayMs,
                serviceTimeoutMs
            );

            StatusEvents["robotDescriptionReceived"] = urdfTransfer.Status["robotDescriptionReceived"];
            StatusEvents["robotNameReceived"] = urdfTransfer.Status["robotNameReceived"];
            StatusEvents["resourceFilesReceived"] = urdfTransfer.Status["resourceFilesReceived"];

            urdfTransfer.Transfer();

            // TODO: Not the best practice here. Handler should observe but not care 
            if (StatusEvents["robotNameReceived"].WaitOne(timeout * 1000))
            {
                // resolve name for the generated GameObject and the local directory for the imported assets
                robotName = urdfTransfer.RobotName;
                localDirectory = urdfTransfer.LocalUrdfDirectory;
            }
            else
            {
                Debug.LogWarning("Robot name was not received before timeout.");
            }

            // No handler-side timeout here: UrdfTransferFromRos owns retries/cancellation
            // and is guaranteed to signal this event exactly once, regardless of outcome.
            StatusEvents["resourceFilesReceived"].WaitOne();

            switch (urdfTransfer.ResourceFileImportProgress)
            {
                case < 0f:
                    Debug.LogWarning("Resource file import failed. Check the console for details.");
                    break;
                case < 1f:
                    Debug.LogWarning("Resource file import incomplete.");
                    break;
                default:
                    Debug.Log("Imported urdf resources to " + localDirectory);
                    break;
            }

            rosSocket.Close();
        }

        // after the editor window has been repainted, schedule the dialog in second repaint
        // 1. first repaint (after import complete)
        // -> 2. render complete state, no dialog yet
        // -> 3. second repaint
        // -> 4. schedule dialog
        public void NotifyGuiRepainted()
        {
            // first repaint after import complete
            if (dialogAlreadyScheduled || !StatusEvents["resourceFilesReceived"].WaitOne(0))
                return;

            // second repaint
            if (!StatusEvents["importComplete"].WaitOne(0))
            {
                StatusEvents["importComplete"].Set();
                return;
            }

            // finally schedule the dialog to be shown in the next repaint
            dialogAlreadyScheduled = true;
            EditorApplication.delayCall += ShowFinalDialog;
        }

        private void ShowFinalDialog()
        {
            AssetDatabase.Refresh();

            switch (ResourceFileImportProgress)
            {
                // failed
                case < 0f:
                    EditorUtility.DisplayDialog("Urdf Assets import failed.",
                        "Some resource files could not be imported. Check the console for details.",
                        "OK");
                    return;

                // incomplete
                case < 1f:
                    EditorUtility.DisplayDialog("Urdf Assets import incomplete.",
                        "Some resource files could not be imported before timeout. Check the console for details.",
                        "OK");
                    return;

                // success
                default:
                    if (EditorUtility.DisplayDialog(
                        "Urdf Assets imported.",
                        "Do you want to generate a " + robotName + " GameObject now?",
                        "Yes", "No"))
                        {
                            Urdf.Editor.UrdfRobotExtensions.Create(Path.Combine(
                                localDirectory,
                                Path.GetFileNameWithoutExtension(robotName) + ".urdf"));
                        } 
                    return;
            }
        }

        public bool CheckForRosConnector()
        {
            rosConnector = GameObject.FindObjectOfType(typeof(RosConnector)) as RosConnector;
            return rosConnector != null;    
        }

        // Aborts any in-progress transfer (retries, pending file requests) and closes the socket.
        // Safe to call from the main/UI thread, e.g. when the editor window is closed.
        public void Cancel()
        {
            cancellationTokenSource?.Cancel();
            rosSocket?.Close();
        }

        public void CreateRosConnector()
        {
            GameObject newGameObject = new GameObject("RosConnectorObject");
            rosConnector = newGameObject.AddComponent<RosConnector>();
        }

        private void OnClosed(object sender, EventArgs e)
        {
            // no point in retrying if the connection is closed, abort 
            if (urdfTransfer != null && !StatusEvents["resourceFilesReceived"].WaitOne(0))
            {
                Debug.Log("RosConnector closed connection, aborting any in-progress transfer.");
                cancellationTokenSource?.Cancel();
            }
            
            StatusEvents["disconnected"].Set();
        }

        private void OnConnected(object sender, EventArgs e)
        {
            StatusEvents["connected"].Set();
        }

        private void ResetStatusEvents()
        {
            dialogAlreadyScheduled = false;

            cancellationTokenSource?.Dispose();
            cancellationTokenSource = new CancellationTokenSource();

            foreach (var manualResetEvent in StatusEvents.Values)
                manualResetEvent.Reset();
        }
    }
}
