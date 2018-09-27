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

using System.IO;
using System.Threading;
using UnityEngine;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class TransferFromRosEditorWindow : EditorWindow
    {
        private static RosConnector.Protocols protocolType;
        private static string address;
        private static int timeout;
        private static string assetPath;
      
        private TransferFromRosHandler transferHandler;

        [MenuItem("RosBridgeClient/Import URDF Assets...")]
        private static void Init()
        {
            TransferFromRosEditorWindow editorWindow = GetWindow<TransferFromRosEditorWindow>();
            editorWindow.minSize = new Vector2(150, 300);
            
            editorWindow.transferHandler = new TransferFromRosHandler();

            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("URDF Asset Importer", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            EditorGUIUtility.labelWidth = 100;
            protocolType = (RosConnector.Protocols)EditorGUILayout.EnumPopup("Protocol", protocolType);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            address = EditorGUILayout.TextField("Address", address);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            timeout = EditorGUILayout.IntField("Timeout [s]", timeout);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            assetPath = EditorGUILayout.TextField("Asset Path", assetPath);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.Space();
            if (GUILayout.Button("Reset to Default", GUILayout.Width(150)))
            {
                DeleteEditorPrefs();
                GetEditorPrefs();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Read Robot Description."))
            {
                SetEditorPrefs();

                Thread rosSocketConnectThread = new Thread(() => transferHandler.BeginRosImport(protocolType, address, timeout, assetPath));
                rosSocketConnectThread.Start();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);

            EditorGUIUtility.labelWidth = 300;

            DrawLabelField("1. rosbridge_server Connected:", "connected");
            DrawLabelField("2. Robot Name Received:", "robotNameReceived");
            DrawLabelField("3. Robot Description Received:", "robotDescriptionReceived");
            DrawLabelField("4. Resource Files Received:", "resourceFilesReceived");
            DrawLabelField("5. rosbridge_server Disconnected:", "disconnected");
            DrawLabelField("6. Import Complete:", "importComplete");
        }

        private void DrawLabelField(string label, string stage)
        {
            GUIStyle guiStyle = new GUIStyle(EditorStyles.textField);
            bool state = transferHandler.StatusEvents[stage].WaitOne(0);
            guiStyle.normal.textColor = state ? Color.green : Color.red;
            EditorGUILayout.LabelField(label, state ? "done" : "open", guiStyle);
        }

        private void OnInspectorUpdate()
        {
            Repaint();

            // some methods can only be called from main thread:
            // We check the status to call the methods at the right step in the process:
            transferHandler.GenerateModelIfReady();
        }

        #region EditorPrefs

        private void OnFocus()
        {
            GetEditorPrefs();
        }

        private void OnLostFocus()
        {
            SetEditorPrefs();
        }

        private void OnDestroy()
        {
            SetEditorPrefs();
        }

        private void DeleteEditorPrefs()
        {
            EditorPrefs.DeleteKey("UrdfImporterProtocolNumber");
            EditorPrefs.DeleteKey("UrdfImporterAddress");
            EditorPrefs.DeleteKey("UrdfImporterAssetPath");
            EditorPrefs.DeleteKey("UrdfImporterTimeout");
        }
        private void GetEditorPrefs()
        {
            protocolType = (RosConnector.Protocols)(EditorPrefs.HasKey("UrdfImporterProtocolNumber") ?
                EditorPrefs.GetInt("UrdfImporterProtocolNumber") : 1);

            address = (EditorPrefs.HasKey("UrdfImporterAddress") ?
                EditorPrefs.GetString("UrdfImporterAddress") :
                "ws://192.168.0.1:9090");

            assetPath = (EditorPrefs.HasKey("UrdfImporterAssetPath") ?
                EditorPrefs.GetString("UrdfImporterAssetPath") :
                Path.Combine(Path.Combine(Path.GetFullPath("."), "Assets"), "Urdf"));

            timeout = (EditorPrefs.HasKey("UrdfImporterTimeout") ?
                EditorPrefs.GetInt("UrdfImporterTimeout") :
                10);
        }
        private void SetEditorPrefs()
        {
            EditorPrefs.SetInt("UrdfImporterProtocol", protocolType.GetHashCode());
            EditorPrefs.SetString("UrdfImporterAddress", address);
            EditorPrefs.SetString("UrdfImporterAssetPath", assetPath);
            EditorPrefs.SetInt("UrdfImporterTimeout", timeout);
        }
        
        #endregion
    }
}
