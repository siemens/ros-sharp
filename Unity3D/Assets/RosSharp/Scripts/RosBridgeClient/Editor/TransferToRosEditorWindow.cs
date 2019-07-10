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

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class TransferToRosEditorWindow : EditorWindow
    {
        private static RosConnector.Protocols protocolType;
        private static string serverUrl = "ws://192.168.56.102:9090";
        private static string urdfPath;
        private static int timeout;
        private static string rosPackage;
        
        private TransferToRosHandler transferHandler;

        private bool showSettings = false;

        [MenuItem("RosBridgeClient/Transfer URDF to ROS...", false, 51)]
        private static void Init()
        {
            TransferToRosEditorWindow editorWindow = GetWindow<TransferToRosEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);

            editorWindow.transferHandler = new TransferToRosHandler();
            editorWindow.GetEditorPrefs();

            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("URDF Transfer (From Unity to ROS)", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            serverUrl = EditorGUILayout.TextField("Server URL", serverUrl);
            EditorGUILayout.EndHorizontal();

            showSettings = EditorGUILayout.Foldout(showSettings, "Settings");
            if (showSettings)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUIUtility.labelWidth = 100;
                protocolType = (RosConnector.Protocols)EditorGUILayout.EnumPopup("Protocol", protocolType);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                urdfPath = EditorGUILayout.TextField("URDF to export", urdfPath);
                if (GUILayout.Button("Select", new GUIStyle(EditorStyles.miniButtonRight) { fixedWidth = 75 }))
                {
                    urdfPath = EditorUtility.OpenFilePanel("Select a URDF file", urdfPath, "urdf");
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                timeout = EditorGUILayout.IntField("Timeout [s]", timeout);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                rosPackage = EditorGUILayout.TextField(
                    new GUIContent("ROS package",
                        "The package where all meshes and resources files will be exported to in ROS."),
                    rosPackage);
                EditorGUILayout.EndHorizontal();
            }

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.Space();
            if (GUILayout.Button("Reset to Default", GUILayout.Width(150)))
            {
                DeleteEditorPrefs();
                GetEditorPrefs();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(10);
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Publish Robot Description"))
            { 
                SetEditorPrefs();
                transferHandler.Transfer(protocolType, serverUrl, timeout, urdfPath, rosPackage);
            }
            EditorGUILayout.EndHorizontal();


            GUILayout.Space(20);
            EditorGUIUtility.labelWidth = 225;
            
            DrawLabelField("Connected: ", "connected");
            DrawLabelField("Robot name published: ", "robotNamePublished");
            DrawLabelField("Robot description published: ", "robotDescriptionPublished");
            DrawLabelField("All resources files published: ", "resourceFilesSent");

            GUILayout.Space(10);
            EditorGUI.BeginDisabledGroup(transferHandler.RosSocket == null || !transferHandler.RosSocket.protocol.IsAlive());
            if (GUILayout.Button("Close Connection"))
                transferHandler.RosSocket?.Close();

            EditorGUI.EndDisabledGroup();
        }

        private void DrawLabelField(string label, string stage)
        {
            GUIStyle guiStyle = new GUIStyle(EditorStyles.textField);
            bool state = transferHandler.StatusEvents[stage].WaitOne(0);
            guiStyle.normal.textColor = state ? Color.green : Color.red;
            EditorGUILayout.LabelField(label, state ? "yes" : "no", guiStyle);
        }

        #region EditorPrefs

        private void OnDestroy()
        {
            SetEditorPrefs();
            transferHandler.RosSocket?.Close();
        }

        private void DeleteEditorPrefs()
        {
            EditorPrefs.DeleteKey("UrdfPublisherProtocolNumber");
            EditorPrefs.DeleteKey("UrdfPublisherServerUrl");
            EditorPrefs.DeleteKey("UrdfPublisherUrdfPath");
            EditorPrefs.DeleteKey("UrdfPublisherTimeout");
            EditorPrefs.DeleteKey("UrdfPublisherRosPackage");
        }

        private void GetEditorPrefs()
        {
            protocolType = (RosConnector.Protocols)(EditorPrefs.HasKey("UrdfPublisherProtocolNumber") ?
                EditorPrefs.GetInt("UrdfPublisherProtocolNumber") : 1);

            serverUrl = (EditorPrefs.HasKey("UrdfPublisherServerUrl") ?
                EditorPrefs.GetString("UrdfPublisherServerUrl") :
                "ws://192.168.0.1:9090");

            urdfPath = (EditorPrefs.HasKey("UrdfPublisherUrdfPath") ?
                EditorPrefs.GetString("UrdfPublisherUrdfPath") :
                Path.Combine(Path.Combine(Path.GetFullPath("."), "Assets"), "Urdf"));

            rosPackage = (EditorPrefs.HasKey("UrdfPublisherRosPackage") ?
                EditorPrefs.GetString("UrdfPublisherRosPackage") :
                "new_package");

            timeout = (EditorPrefs.HasKey("UrdfPublisherTimeout") ?
                EditorPrefs.GetInt("UrdfPublisherTimeout") :
                10);
        }

        private void SetEditorPrefs()
        {
            EditorPrefs.SetInt("UrdfPublisherProtocolNumber", protocolType.GetHashCode());
            EditorPrefs.SetString("UrdfPublisherServerUrl", serverUrl);
            EditorPrefs.SetString("UrdfPublisherUrdfPath", urdfPath);
            EditorPrefs.SetString("UrdfPublisherRosPackage", rosPackage);
            EditorPrefs.SetInt("UrdfPublisherTimeout", timeout);
        }

        #endregion
    }
}