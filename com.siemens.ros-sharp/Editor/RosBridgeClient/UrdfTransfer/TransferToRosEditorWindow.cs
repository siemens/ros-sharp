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

#if UNITY_EDITOR

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class TransferToRosEditorWindow : EditorWindow
    {
        private static string robotNameParameter;
        private bool rosConnectorFound = false;
        private static string urdfPath;
        private static string rosPackage;
        private TransferToRosHandler transferHandler;
        private bool showSettings = true;
        private static string defaultRosPackage = "urdf_export_test:param_name";

#if ROS2
        private static string defautRobotNameParameter = "r2d2:file_server2";
        private static string hintRobotNameParameter = "Syntax:\n<node_name>:<param_name>\nExample usage:\n<robot_name>:<package_name>";
        
#else
        private static string defautRobotNameParameter = "/robot/name";
        private static string hintRobotNameParameter = "Syntax:\n<param_name>\nExample usage:\n/robot/name";
#endif

        [MenuItem("RosBridgeClient/Transfer URDF to ROS...", false, 51)]
        private static void Init()
        {
            TransferToRosEditorWindow editorWindow = GetWindow<TransferToRosEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);

            editorWindow.transferHandler = new TransferToRosHandler();
            // Check if a RosConnector is already present
            editorWindow.rosConnectorFound = editorWindow.transferHandler.CheckForRosConnector();

            editorWindow.GetEditorPrefs();
            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("URDF Transfer (From Unity to ROS)", EditorStyles.boldLabel);
            
            GUILayout.Space(15);
            HandleRosConnectorButton();
            GUILayout.Space(15);

            showSettings = EditorGUILayout.Foldout(showSettings, "Settings");
            if (showSettings)
            {
                EditorGUILayout.BeginHorizontal();
                robotNameParameter = EditorGUILayout.TextField(new GUIContent("Robot name parameter", hintRobotNameParameter), robotNameParameter);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                urdfPath = EditorGUILayout.TextField("URDF to export", urdfPath);
                if (GUILayout.Button("Select", new GUIStyle(EditorStyles.miniButtonRight) { fixedWidth = 75 }))
                {
                    urdfPath = EditorUtility.OpenFilePanel("Select a URDF file", urdfPath, "urdf");
                }
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
                transferHandler.Transfer(urdfPath, rosPackage, robotNameParameter);
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
            if (GUILayout.Button("Close Connection")) {
                Debug.Log("Closing connection to ROS...");
                transferHandler.RosSocket?.Close();
                }

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
            EditorPrefs.DeleteKey("UrdfImporterRobotName");
            EditorPrefs.DeleteKey("UrdfPublisherUrdfPath");
            EditorPrefs.DeleteKey("UrdfPublisherRosPackage");
        }

        private void GetEditorPrefs()
        {
            robotNameParameter = (EditorPrefs.HasKey("UrdfImporterRobotName") ?
                EditorPrefs.GetString("UrdfImporterRobotName") :
                defautRobotNameParameter);

            urdfPath = (EditorPrefs.HasKey("UrdfPublisherUrdfPath") ?
                EditorPrefs.GetString("UrdfPublisherUrdfPath") :
                Path.Combine(Path.Combine(Path.GetFullPath("."), "Assets"), "Urdf"));

            rosPackage = (EditorPrefs.HasKey("UrdfPublisherRosPackage") ?
                EditorPrefs.GetString("UrdfPublisherRosPackage") :
                defaultRosPackage);
        }

        private void SetEditorPrefs()
        {
            EditorPrefs.SetString("UrdfImporterRobotName", robotNameParameter);
            EditorPrefs.SetString("UrdfPublisherUrdfPath", urdfPath);
            EditorPrefs.SetString("UrdfPublisherRosPackage", rosPackage);
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

#endif