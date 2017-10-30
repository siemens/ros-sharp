﻿/*
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

using System.IO;
using System.Threading;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using RosSharp.RosBridgeClient;

namespace RosSharp.UrdfImporter
{

    public class UrdfImporterEditorWindow : EditorWindow
    {
        private string address = "ws://192.168.0.1:9090";
        private string timeout = "10";
        private string urdfAssetPath = Path.GetFullPath(".") + "\\Assets\\Urdf\\";

        private Thread rosSocketConnectThread;
        private Thread urdfImportThread;
        private RosBridgeClient.UrdfImporter urdfImporter;

        private Dictionary<string, ManualResetEvent> status = new Dictionary<string, ManualResetEvent>
    {
        { "connected", new ManualResetEvent(false) },
        { "robotNameReceived",new ManualResetEvent(false) },
        { "robotDescriptionReceived", new ManualResetEvent(false) },
        { "resourceFilesReceived", new ManualResetEvent(false) },
        { "disconnected", new ManualResetEvent(false) },
        { "databaseRefreshStarted", new ManualResetEvent(false) },
        { "databaseRefreshed", new ManualResetEvent(false) },
        { "importModelDialogShown", new ManualResetEvent(false) },
    };

        [MenuItem("RosBridgeClient/Import URDF Assets...")]
        private static void Init()
        {
            UrdfImporterEditorWindow editorWindow = GetWindow<UrdfImporterEditorWindow>();
            editorWindow.minSize = new Vector2(150, 300);
            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("ROSbridge WebSocket", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            EditorGUIUtility.labelWidth = 100;
            address = EditorGUILayout.TextField("Address", address);
            timeout = EditorGUILayout.TextField("Timeout [s]", timeout);
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginHorizontal();
            urdfAssetPath = EditorGUILayout.TextField("Asset Path", urdfAssetPath);
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Read Robot Description."))
            {
                rosSocketConnectThread = new Thread(() => rosSocketConnect());
                rosSocketConnectThread.Start();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);

            EditorGUIUtility.labelWidth = 300;

            drawLabelField("1. ROSbridge_server Connected:", "connected");
            drawLabelField("2. Robot Name Received:", "robotNameReceived");
            drawLabelField("3. Robot Description Received:", "robotDescriptionReceived");
            drawLabelField("4. Resource Files Received:", "resourceFilesReceived");
            drawLabelField("5. ROSBridge_server Disconnected:", "disconnected");
            drawLabelField("6. Asset Database Refresh Completed:", "databaseRefreshed");
        }

        private void drawLabelField(string label, string stage)
        {
            GUIStyle guiStyle = new GUIStyle(EditorStyles.textField);
            bool state = status[stage].WaitOne(0);
            guiStyle.normal.textColor = state ? Color.green : Color.red;
            EditorGUILayout.LabelField(label, state ? "done" : "open", guiStyle);
        }

        private void rosSocketConnect()
        {
            // intialize
            foreach (ManualResetEvent manualResetEvent in status.Values)
                manualResetEvent.Reset();

            // connect to ROSbridge
            RosSocket rosSocket = new RosSocket(address);
            status["connected"].Set();

            // setup urdfImporter
            urdfImporter = new RosBridgeClient.UrdfImporter(rosSocket, urdfAssetPath);
            status["robotNameReceived"] = urdfImporter.Status["robotNameReceived"];
            status["robotDescriptionReceived"] = urdfImporter.Status["robotDescriptionReceived"];
            status["resourceFilesReceived"] = urdfImporter.Status["resourceFilesReceived"];
            Thread urdfImportThread = new Thread(() => urdfImporter.Import());
            urdfImportThread.Start();

            // import URDF assets:
            if (status["resourceFilesReceived"].WaitOne(int.Parse(timeout) * 1000))
                Debug.Log("Imported urdf resources to " + urdfImporter.LocalDirectory);
            else
                Debug.LogWarning("Not all resource files have been received before timeout.");

            // close the ROSBridge socket
            rosSocket.Close();
            status["disconnected"].Set();
        }

        private void OnInspectorUpdate()
        {
            // some methods can only be called from main thread:
            // We check the status to call the methods at the right step in the process:

            Repaint();

            // import Model
            if (status["databaseRefreshed"].WaitOne(0) && !status["importModelDialogShown"].WaitOne(0))
            {
                status["importModelDialogShown"].Set();
                if (EditorUtility.DisplayDialog(
                    "Urdf Assets imported.",
                    "Do you want to generate a " + urdfImporter.robotName + " GameObject now?",
                    "Yes", "No"))
                {
                    RobotCreator.Create(Path.Combine(urdfImporter.LocalDirectory, "robot_description.urdf"));
                }
            }

            // refresh Asset Database 
            if (status["resourceFilesReceived"].WaitOne(0) && !status["databaseRefreshed"].WaitOne(0))
            {
                status["databaseRefreshStarted"].Set();
                AssetDatabase.Refresh();
                status["databaseRefreshed"].Set();
            }
        }
    }
}