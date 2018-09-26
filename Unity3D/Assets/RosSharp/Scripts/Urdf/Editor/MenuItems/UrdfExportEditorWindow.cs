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

namespace RosSharp.Urdf.Editor
{
    public class UrdfExportEditorWindow : EditorWindow
    {
        public UrdfRobot urdfRobot;
        public string exportRoot = "";
        public string subfolder = "";
        public int selectedSubfolder;

        private static string[] subfolderOptions = { "Export URDF to root folder", "Export URDF to the following subfolder:" };
        
        private void OnGUI()
        {
            //Styles definitions
            GUIStyle titleStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 13
            };
            GUIStyle buttonStyle = new GUIStyle(EditorStyles.miniButtonRight) {fixedWidth = 75};

            //Window title
            GUILayout.Space(10);
            GUILayout.Label("Export " + urdfRobot.gameObject.name + " to URDF", titleStyle);

            //Select export root folder
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal("HelpBox");
            exportRoot = EditorGUILayout.TextField(
                new GUIContent("Export root folder", "Corresponds to ROS package root folder."),
                exportRoot);
            if (GUILayout.Button("Select", buttonStyle))
            {
                exportRoot = EditorUtility.OpenFolderPanel("Select export root folder", exportRoot, "");
            }

            EditorGUILayout.EndHorizontal();

            //Select subfolder
            GUILayout.Space(5);
            selectedSubfolder =
                GUILayout.SelectionGrid(selectedSubfolder, subfolderOptions, 1, EditorStyles.radioButton);

            EditorGUI.BeginDisabledGroup(selectedSubfolder != 1);

            EditorGUILayout.BeginHorizontal();
            GUILayout.Space(30);
            EditorGUILayout.BeginHorizontal("HelpBox");
            subfolder = EditorGUILayout.TextField(
                new GUIContent("Subfolder", "Corresponds to URDF subfolder in ROS package."),
                subfolder);
            if (GUILayout.Button("Select", buttonStyle))
            {
                string subfolderPath = EditorUtility.OpenFolderPanel(
                    "Select export destination for robot asset files (such as meshes, images, etc)",
                    exportRoot,
                    "");

                subfolder = subfolderPath.Contains(exportRoot) ? subfolderPath.Substring(exportRoot.Length) : "";
            }

            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndHorizontal();

            EditorGUI.EndDisabledGroup();

            //Choose STL export type
            GUILayout.Space(10);
            EditorGUILayout.BeginHorizontal();
            StlWriter.fileType =
                (StlWriter.FileType) EditorGUILayout.EnumPopup("Export new meshes to", StlWriter.fileType);
            EditorGUILayout.LabelField("   STL files");
            EditorGUILayout.EndHorizontal();

            //Export Robot button
            GUILayout.Space(10);
            if (GUILayout.Button("Export Robot"))
            {
                if (exportRoot == "" || !Directory.Exists(exportRoot))
                    EditorUtility.DisplayDialog("Export Error",
                        "Export root folder must be defined and folder must exist.", "Ok");
                else
                {
                    if (selectedSubfolder == 0)
                        subfolder = "";
                    else
                        subfolder = subfolder.TrimStart(Path.DirectorySeparatorChar)
                            .TrimStart(Path.AltDirectorySeparatorChar);

                    urdfRobot.ExportRobotToUrdf(exportRoot, subfolder);
                    SetEditorPrefs();
                    Close();
                }
            }
        }

        public void GetEditorPrefs()
        {
            exportRoot = EditorPrefs.HasKey("UrdfExportRoot") ?
                EditorPrefs.GetString("UrdfExportRoot") : "";
            
            subfolder = EditorPrefs.HasKey("UrdfExportSubfolder") ?
                EditorPrefs.GetString("UrdfExportSubfolder") : "";
            
            selectedSubfolder = EditorPrefs.HasKey("UrdfExportSubfolderOption") ?
                EditorPrefs.GetInt("UrdfExportSubfolderOption") : 0;
        }
        private void SetEditorPrefs()
        {
            EditorPrefs.SetString("UrdfExportRoot", exportRoot);
            EditorPrefs.SetString("UrdfExportSubfolder", subfolder);
            EditorPrefs.SetInt("UrdfExportSubfolderOption", selectedSubfolder);
        }
    }
}
