/*
© Siemens AG, 2019
Author: Sifan Ye (sifan.ye@siemens.com)

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

using System.Collections.Concurrent;
using System.IO;
using System.Threading;

using UnityEngine;
using UnityEditor;

using RosSharp.RosBridgeClient.MessageTypes.FileServer;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient.RosFileTransfer
{
    public class FileTransferFromRosEditorWindow : EditorWindow
    {
        private string serverURL = "ws://192.168.137.195:9090";
        private Protocol protocol = Protocol.WebSocketNET;
        private RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON;

        private float timeout = 3f;
        private float timestep = 0.1f;

        private FileTransferType type = FileTransferType.PACKAGE;

        private string resourceIdentifier = "file_server";

        public string[] extensions = new string[0];
        private SerializedObject serializedObject;
        private SerializedProperty serializedProperty;

        private string outPath = Path.Combine(System.Environment.CurrentDirectory, "Assets");

        private FileTransferFromRosUnityClient client;

        [MenuItem("RosBridgeClient/ROS File Transfer/From ROS...", false, 70)]
        public static void OpenWindow()
        {
            FileTransferFromRosEditorWindow window = GetWindow<FileTransferFromRosEditorWindow>(false, "ROS File Transfer", true);
            window.minSize = new Vector2(600, 250);
            window.Show();
        }

        private void Awake()
        {
            
        }

        private void OnGUI()
        {
            GUILayout.Label("File Transfer From ROS", EditorStyles.boldLabel);

            GUILayout.Space(4);

            EditorGUILayout.BeginHorizontal();
            serverURL = EditorGUILayout.TextField("ROS File Server Address", serverURL);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            protocol = (Protocol)EditorGUILayout.EnumPopup("Protocol", protocol);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            serializer = (RosSocket.SerializerEnum)EditorGUILayout.EnumPopup("Serializer", serializer);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            timeout = EditorGUILayout.FloatField("Timeout (seconds):", timeout);
            timestep = EditorGUILayout.FloatField("Timestep (seconds):", timestep);
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(16);

            EditorGUILayout.BeginHorizontal();
            type = (FileTransferType)EditorGUILayout.EnumPopup("File Transfer Type", type);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            resourceIdentifier = EditorGUILayout.TextField("Resource Identifier:", resourceIdentifier);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            ScriptableObject target = this;
            serializedObject = new SerializedObject(target);
            serializedObject.Update();
            serializedProperty = serializedObject.FindProperty("extensions");
            EditorGUILayout.PropertyField(serializedProperty, true);
            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            outPath = EditorGUILayout.TextField("Output Location", outPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                outPath = EditorUtility.OpenFolderPanel("Select Folder...", "", "");
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(16);

            if (GUILayout.Button("Initiate File Transfer"))
            {
                FileTransferGoal goal = new FileTransferGoal();
                goal.type = (byte)type;
                goal.identifier = resourceIdentifier;
                if (extensions == null)
                {
                    goal.extensions = new string[0];
                }
                else
                {
                    goal.extensions = extensions;
                }

                client = new FileTransferFromRosUnityClient(new FileTransferAction(), outPath, serverURL, protocol, serializer, timeout, timestep, false);
                client.Start();
                client.UnitySetGoal(goal);

                Thread.Sleep((int)(timestep * 1000));

                // Wait for server
                ManualResetEvent isWaitingForServer = new ManualResetEvent(false);
                ManualResetEvent shouldWaitForServer = new ManualResetEvent(false);
                isWaitingForServer.Set();
                shouldWaitForServer.Set();
                Thread waitForServer = new Thread(() => client.UnityWaitForServer(isWaitingForServer, shouldWaitForServer));
                waitForServer.Start();
                while (isWaitingForServer.WaitOne(0))
                {
                    if (EditorUtility.DisplayCancelableProgressBar("Waiting for action server...", "This may take a while...", 1f))
                    {
                        shouldWaitForServer.Reset();
                        waitForServer.Join();
                        EditorUtility.ClearProgressBar();
                        return;
                    }
                    Thread.Sleep((int)(timestep * 1000));
                }

                // Send goal
                ManualResetEvent isSendingGoal = new ManualResetEvent(false);
                isSendingGoal.Set();
                Thread sendGoal = new Thread(() => client.UnitySendGoal(isSendingGoal));
                sendGoal.Start();
                while (isSendingGoal.WaitOne(0))
                {
                    if (EditorUtility.DisplayCancelableProgressBar("Sending goal...", "Failed to come up with a witty message", 1f))
                    {
                        client.CancelGoal();
                        EditorUtility.ClearProgressBar();
                        return;
                    }
                    Thread.Sleep((int)(timestep * 1000));
                }
                sendGoal.Join();
                EditorUtility.ClearProgressBar();

                // Write Files
                bool isInterrupted = false;
                ConcurrentQueue<FileTransferFeedback> files = client.UnityGetFiles();
                while (!client.UnityIsResultReceived() || !files.IsEmpty)
                {
                    if (files.TryDequeue(out FileTransferFeedback file))
                    {
                        string completeOutPath = client.UnityGetCompleteOutPath(file);
                        File.WriteAllBytes(completeOutPath, file.content);
                        float progress = (float)(file.number) / (float)(file.count);
                        if (EditorUtility.DisplayCancelableProgressBar("Saving files...(" + file.number + "/" + file.count + ")", "\"What's Secure Copy Protocol Foundation?\"", progress))
                        {
                            client.UnityCancelGoal();
                            isInterrupted = true;
                            break;
                        }
                    }
                    Thread.Sleep((int)(timestep * 1000));
                }
                EditorUtility.ClearProgressBar();

                // Flush Queue
                if (!isInterrupted)
                {
                    while (!files.IsEmpty)
                    {
                        if (files.TryDequeue(out FileTransferFeedback file))
                        {
                            string completeOutPath = client.UnityGetCompleteOutPath(file);
                            File.WriteAllBytes(completeOutPath, file.content);
                            float progress = (float)(file.number) / (float)(file.count);
                            EditorUtility.DisplayProgressBar("Flushing queue...(" + file.number + "/" + file.count + ")", "*Insert toilet flushing noises*", progress);
                        }
                    }
                }
                EditorUtility.ClearProgressBar();

                // Done
                if (isInterrupted)
                {
                    EditorUtility.DisplayDialog(
                        title: "File transfer interrupted",
                        message: "Output at: " + outPath,
                        ok: "OK");
                }
                else
                {
                    EditorUtility.DisplayDialog(
                        title: "File transfer complete",
                        message: "Output at: " + outPath,
                        ok: "Thank you!");
                }
                client.Stop();
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void OnDestroy()
        {
            if (client != null)
            {
                client.Stop();
            }
        }
    }
}
