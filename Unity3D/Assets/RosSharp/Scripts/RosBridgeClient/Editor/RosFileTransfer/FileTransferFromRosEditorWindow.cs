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

using System.IO;
using System.Threading;
using System.Threading.Tasks;

using UnityEngine;
using UnityEditor;

using RosSharp.RosBridgeClient.MessageTypes.FileServer;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient.RosFileTransfer
{
    public class FileTransferFromRosEditorWindow : EditorWindow
    {
        private string serverURL = "ws://192.168.137.195:9090";
        private Protocol protocol;
        private RosSocket.SerializerEnum serializer;

        private float timeout;
        private float timestep;

        private FileTransferType type;

        private string resourceIdentifier;

        public string[] extensions;
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
                FileTransferAction action = new FileTransferAction();
                FileTransferGoal goal = action.action_goal.goal;
                goal.type = (byte)type;
                goal.identifier = resourceIdentifier;
                goal.extensions = extensions;

                client = new FileTransferFromRosUnityClient(action, outPath, serverURL, protocol, serializer, timeout, timestep, false);
                client.Start();

                Thread.Sleep((int)(timestep * 1000));

                ManualResetEvent isWaitingForServer = new ManualResetEvent(false);
                ManualResetEvent shouldWaitForServer = new ManualResetEvent(false);
                isWaitingForServer.Set();
                shouldWaitForServer.Set();
                Task waitForServer = new Task(() => client.UnityWaitForServer(isWaitingForServer, shouldWaitForServer));
                waitForServer.Start();
                while (isWaitingForServer.WaitOne(0))
                {
                    if (EditorUtility.DisplayCancelableProgressBar("Waiting for action server...", "This may take a while...", 1f))
                    {
                        shouldWaitForServer.Reset();
                        waitForServer.Wait();
                        client.Stop();
                        EditorUtility.ClearProgressBar();
                    }
                }
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
