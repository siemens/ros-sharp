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

namespace RosSharp.Urdf.Export
{
    public class UrdfExportEditorWindow : EditorWindow
    {
        public UrdfRobot urdfRobot;
        public string exportRoot = "";
        public string exportDestination = "";
        public bool useExportDestination;
        public bool generateJointNames;

        private void OnGUI()
        {
            //Styles definitions
            GUIStyle titleStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 13
            };
            GUIStyle popupStyle = new GUIStyle(EditorStyles.popup) { fixedWidth = 100 };
            GUIStyle buttonStyle = new GUIStyle(EditorStyles.miniButtonRight){ fixedWidth = 100};

            //Window title
            GUILayout.Space(10);
            GUILayout.Label("Export " + urdfRobot.gameObject.name + "to URDF", titleStyle);

            //Select export root folder
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal("HelpBox");
            exportRoot = EditorGUILayout.TextField("Export root folder", exportRoot); 
            if (GUILayout.Button("Select folder", buttonStyle))
            {
                exportRoot = EditorUtility.OpenFolderPanel("Select package root of exported robot", exportRoot, "");
            }
            EditorGUILayout.EndHorizontal();

            //Select export destination folder
            GUILayout.Space(10);
            useExportDestination = GUILayout.Toggle(useExportDestination, " Different export destination");
            EditorGUILayout.HelpBox("Enable this option in order to select a subfolder of the export root folder" +
                                    " where all robot assets will be exported. In the URDF, the package path for resources " +
                                    "like meshes and textures will be relative to the export root folder.", MessageType.Info);
            if (useExportDestination)
            {
                EditorGUILayout.BeginHorizontal("HelpBox");
                exportDestination = EditorGUILayout.TextField("Export destination", exportDestination);
                if (GUILayout.Button("Select folder", buttonStyle))
                {
                    exportDestination = EditorUtility.OpenFolderPanel(
                        "Select export destination for robot asset files (such as meshes, images, etc)",
                        exportRoot,
                        "");
                }
                EditorGUILayout.EndHorizontal();
            }

            //Choose whether to generate joint names
            GUILayout.Space(10);
            generateJointNames = GUILayout.Toggle(generateJointNames, " Generate unique joint names");
            if (generateJointNames)
                EditorGUILayout.HelpBox("This will overwrite all user defined joint names with automatically generated joint " +
                                        "names, derived from the parent and child links.", MessageType.Info);

            //Choose STL export type
            GUILayout.Space(5);
            StlWriter.fileType = (StlWriter.FileType)EditorGUILayout.EnumPopup("Export new meshes to", StlWriter.fileType, popupStyle);

            //Export Robot button
            GUILayout.Space(10);
            if (GUILayout.Button("Export Robot"))
            {
                if (generateJointNames)
                    urdfRobot.GenerateUniqueJointNames();

                if (exportRoot == "" || !Directory.Exists(exportRoot))
                    EditorUtility.DisplayDialog("Export Error", "Export root folder must be defined and folder must exist.", "Ok");
                else
                {
                    if (!useExportDestination)
                        exportDestination = exportRoot;

                    urdfRobot.ExportRobotToUrdf(exportRoot, exportDestination);

                    Close();
                }
            }
        }
    }
}
