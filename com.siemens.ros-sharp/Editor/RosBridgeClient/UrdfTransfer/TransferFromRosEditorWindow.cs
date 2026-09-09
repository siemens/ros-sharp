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

* Removed additional RosConnector instance. Urdf transfer is now handled by the existing RosConnector component.
* If no RosConnector component is present in the scene, one can be created by pressing the button.
* RosConnector specific input fields have been removed as they are no longer required.
* Robot name parameter input field added.
* The 'Reset to Default' button now behaves according to the selected ROS version (from the RosConnector component). 
* Added GUI hints for parameter syntax. 
    (C) Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

* Fix misleading robot name parameter input field.
* Reorder the status fields to match the order of the transfer process.
* Add progress bar for resource file import status.
* Trigger transfer handler to pop final dialog only after the editor window has been repainted, 
instead of constantly checking for the import complete status in the update loop.
* Add new input fields for max retries, retry delay, and service timeout.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

using System.IO;
using System.Threading;
using UnityEngine;
using UnityEditor;

namespace RosSharp.RosBridgeClient
{
    public class TransferFromRosEditorWindow : EditorWindow
    {
        private static string robotNameParameter;
        private static string urdfParameter;
        private static string assetPath;
        private static int maxRetries;
        private static int retryDelayMs;
        private static int serviceTimeoutMs;
      
        private TransferFromRosHandler transferHandler;

        private bool showSettings = true;
        private bool rosConnectorFound = false;

#if ROS2
        private static string defaultRobotName = "file_server:robot_name";
        private static string defaultUrdfParameter = "robot_state_publisher:robot_description";
        private static string hintRobotName = "Syntax:\n<node_name>:<param_name>\nExample usage:\npkg_name:robot_name";
        private static string hintUrdfParameter = "Syntax:\n<node_name>:<param_name>\nExample usage:\nrobot_state_publisher:robot_description";
#else
        private static string defaultRobotName = "/robot/name";
        private static string defaultUrdfParameter = "/robot_description";
        private static string hintRobotName = "Syntax:\n<param_name>\nExample usage:\n/robot/name";
        private static string hintUrdfParameter = "Syntax:\n<param_name>\nExample usage:\n/robot_description";
#endif
        private static int defaultMaxRetries = 3;
        private static int defaultRetryDelayMs = 200;
        private static int defaultServiceTimeoutMs = 8000;

        private static string hintMaxRetries = "Maximum number of retries for the service call in case of failure.";
        private static string hintRetryDelayMs = "Delay in milliseconds between retries for the service call.";
        private static string hintServiceTimeoutMs = "Timeout in milliseconds for each service call.";

        [MenuItem("RosBridgeClient/Transfer URDF from ROS...", false, 50)]
        private static void Init()
        {
            TransferFromRosEditorWindow editorWindow = GetWindow<TransferFromRosEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);

            editorWindow.transferHandler = new TransferFromRosHandler();

            // Check if a RosConnector is already present
            editorWindow.rosConnectorFound = editorWindow.transferHandler.CheckForRosConnector();

            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("URDF Transfer (From ROS to Unity)", EditorStyles.boldLabel);
            
            GUILayout.Space(15);
            HandleRosConnectorButton();
            GUILayout.Space(15);

            showSettings = EditorGUILayout.Foldout(showSettings, "Settings");
            if (showSettings)
            {
                EditorGUILayout.BeginHorizontal();
                robotNameParameter = EditorGUILayout.TextField(new GUIContent("Robot Name Parameter", hintRobotName), robotNameParameter);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                urdfParameter = EditorGUILayout.TextField(new GUIContent("URDF Parameter", hintUrdfParameter), urdfParameter);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                assetPath = EditorGUILayout.TextField("Asset Path", assetPath);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                serviceTimeoutMs = EditorGUILayout.IntField(new GUIContent("Service Timeout (ms)", hintServiceTimeoutMs), serviceTimeoutMs);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                maxRetries = EditorGUILayout.IntField(new GUIContent("Max Retries", hintMaxRetries), maxRetries);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                retryDelayMs = EditorGUILayout.IntField(new GUIContent("Retry Delay (ms)", hintRetryDelayMs), retryDelayMs);
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

            GUILayout.Space(20);
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Read Robot Description"))
            {
                SetEditorPrefs();

                Thread rosSocketConnectThread = new Thread(() => transferHandler.TransferUrdf(
                    assetPath, 
                    urdfParameter,
                    robotNameParameter,
                    maxRetries,
                    retryDelayMs,
                    serviceTimeoutMs)
                );

                rosSocketConnectThread.Start();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);

            EditorGUIUtility.labelWidth = 300;

            DrawLabelField("Connected:", "connected");
            DrawLabelField("Robot Description Received:", "robotDescriptionReceived");
            DrawLabelField("Robot Name Received:", "robotNameReceived");
            DrawProgressField("Resource Files Received:", transferHandler.ResourceFileImportProgress);
            DrawLabelField("Disconnected:", "disconnected");
            DrawLabelField("Import Complete:", "importComplete");

            // Only notify the handler once this frame has actually been drawn (repainted)
            if (Event.current.type == EventType.Repaint)
                transferHandler.NotifyGuiRepainted();
        }

        private void DrawLabelField(string label, string stage)
        {
            GUIStyle guiStyle = new GUIStyle(EditorStyles.textField);
            guiStyle.alignment = TextAnchor.MiddleCenter;
            bool state = transferHandler.StatusEvents[stage].WaitOne(0);
            guiStyle.normal.textColor = state ? Color.green : Color.red;
            EditorGUILayout.LabelField(label, state ? "done" : "open", guiStyle);
        }
        private void DrawProgressField(string label, float progress)
        {
            Rect rowRect = EditorGUILayout.GetControlRect(false, EditorGUIUtility.singleLineHeight);
            Rect progressRect = EditorGUI.PrefixLabel(rowRect, new GUIContent(label));

            bool failed = progress < 0f;
            bool notStarted = progress <= 0f && !failed;
            float clamped = Mathf.Clamp01(progress);

            string statusText = failed ? "failed" : notStarted ? "open" : $"{clamped:P0}";

            Color prevColor = GUI.color;
            GUI.color = failed ? new Color(0.8f, 0.2f, 0.2f)
                : notStarted ? new Color(0.6f, 0.15f, 0.15f)
                : new Color(0.2f, 0.8f, 0.2f);
                
            EditorGUI.ProgressBar(progressRect, failed ? 1f : clamped, statusText);
            GUI.color = prevColor;
        }

        private void OnInspectorUpdate()
        {
            Repaint();
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
            transferHandler?.Cancel();
        }

        private void DeleteEditorPrefs()
        {
            EditorPrefs.DeleteKey("UrdfImporterAssetPath");
            EditorPrefs.DeleteKey("UrdfImporterUrdfParameter");
            EditorPrefs.DeleteKey("UrdfImporterRobotName");
            EditorPrefs.DeleteKey("UrdfImporterMaxRetries");
            EditorPrefs.DeleteKey("UrdfImporterRetryDelayMs");
            EditorPrefs.DeleteKey("UrdfImporterServiceTimeoutMs");
        }
        private void GetEditorPrefs()
        {
            assetPath = (EditorPrefs.HasKey("UrdfImporterAssetPath") ?
                EditorPrefs.GetString("UrdfImporterAssetPath") :
                Path.Combine(Path.Combine(Path.GetFullPath("."), "Assets"), "Urdf"));

            urdfParameter = (EditorPrefs.HasKey("UrdfImporterUrdfParameter") ?
                EditorPrefs.GetString("UrdfImporterUrdfParameter") :
                defaultUrdfParameter);

            robotNameParameter = (EditorPrefs.HasKey("UrdfImporterRobotName") ?
                EditorPrefs.GetString("UrdfImporterRobotName") :
                defaultRobotName);

            maxRetries = (EditorPrefs.HasKey("UrdfImporterMaxRetries") ?
                EditorPrefs.GetInt("UrdfImporterMaxRetries") :
                defaultMaxRetries);

            retryDelayMs = (EditorPrefs.HasKey("UrdfImporterRetryDelayMs") ?
                EditorPrefs.GetInt("UrdfImporterRetryDelayMs") :
                defaultRetryDelayMs);

            serviceTimeoutMs = (EditorPrefs.HasKey("UrdfImporterServiceTimeoutMs") ?
                EditorPrefs.GetInt("UrdfImporterServiceTimeoutMs") :
                defaultServiceTimeoutMs);

        }
        private void SetEditorPrefs()
        {
            EditorPrefs.SetString("UrdfImporterAssetPath", assetPath);
            EditorPrefs.SetString("UrdfImporterUrdfParameter", urdfParameter);
            EditorPrefs.SetString("UrdfImporterRobotName", robotNameParameter);
            EditorPrefs.SetInt("UrdfImporterMaxRetries", maxRetries);
            EditorPrefs.SetInt("UrdfImporterRetryDelayMs", retryDelayMs);
            EditorPrefs.SetInt("UrdfImporterServiceTimeoutMs", serviceTimeoutMs);
        }
        
        #endregion

        private void HandleRosConnectorButton() {       
            // Display the "ROS Connector Status:" text
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("ROS Connector Status: ", GUILayout.Width(200));

            // Set the button text and color based on whether a RosConnector was found
            string buttonText = rosConnectorFound ? "Available" : "Not found. Create one?";
            Color buttonColor = rosConnectorFound ? Color.green : Color.red;

            // Set the GUI color
            GUI.color = buttonColor;

            // Enable or disable the button based on whether a RosConnector was found
            GUI.enabled = !rosConnectorFound;

            // Display the button
            if (GUILayout.Button(buttonText, GUILayout.ExpandWidth(true)))
            {
                // If the button is clicked, create a new RosConnector
                transferHandler.CreateRosConnector();
                rosConnectorFound = transferHandler.CheckForRosConnector();
                RosConnectorEditor.ShowRosConnectorEditorInspector(transferHandler.rosConnector.gameObject);

                // Update the button text and color based on the new result
                buttonText = rosConnectorFound ? "Available" : "Not found. Create one?";
                buttonColor = rosConnectorFound ? Color.green : Color.red;
                GUI.color = buttonColor;
                GUI.enabled = !rosConnectorFound;
            }

            // Reset the GUI color and enabled state
            GUI.color = Color.white;
            GUI.enabled = true;

            EditorGUILayout.EndHorizontal();
        }
    }
}
