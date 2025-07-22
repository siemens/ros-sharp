/*
© Siemens AG, 2019

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

using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public abstract class SingleAutoGenEditorWindow : EditorWindow
    {
        [SerializeField]
        private static string lastFileDirectory = string.Empty;
        [SerializeField]
        private static string lastOutputDirectory = string.Empty;

        private string inFilePath = "";
        private string outFilePath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosSharpMessages");
        private string rosPackageName = "";
        protected bool toggleROS2 = true;

        protected abstract string GenerationType { get; }
        protected abstract string FileExtension { get; }

        protected virtual void OnGUI()
        {
            GUILayout.Label("Single " + GenerationType + " auto generation", EditorStyles.boldLabel);

            toggleROS2 = GUILayout.Toggle(toggleROS2, "ROS2 Message");

            EditorGUILayout.BeginHorizontal();
            inFilePath = EditorGUILayout.TextField("Input File Path", inFilePath);

            if (GUILayout.Button("Browse File...", GUILayout.Width(120)))
            {
                inFilePath = EditorUtility.OpenFilePanel("Select " + GenerationType + " File...", lastFileDirectory, FileExtension);
                if (!inFilePath.Equals(""))
                {
                    string[] directoryLevels = inFilePath.Split('/');
                    rosPackageName = directoryLevels[directoryLevels.Length - 3];
                }
            }
            EditorGUILayout.EndHorizontal();

            rosPackageName = EditorGUILayout.TextField("ROS Package Name:", rosPackageName);

            EditorGUILayout.BeginHorizontal();
            outFilePath = EditorGUILayout.TextField("Output File Location", outFilePath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(120)))
            {
                outFilePath = EditorUtility.OpenFolderPanel("Select Folder...", lastOutputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("Generate"))
            {
                if (inFilePath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Input File Path Required",
                        message: "The input file path is empty. Please specify a valid input file.",
                        ok: "OK");
                }
                else
                {
                    lastFileDirectory = inFilePath;
                    lastOutputDirectory = outFilePath;
                    try
                    {
                        List<string> warnings = Generate(inFilePath, outFilePath, toggleROS2, rosPackageName);
                        AssetDatabase.Refresh();
                        if (warnings.Count == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: "Code Generation Successful",
                                message: $"Code generation completed successfully. Output location: {outFilePath}",
                                ok: "OK");
                        }
                        else
                        {
                            foreach (string w in warnings)
                            {
                                Debug.LogWarning(w);
                            }
                            EditorUtility.DisplayDialog(
                                title: "Code Generation Completed with Warnings",
                                message: $"Output location: {outFilePath}\n{warnings.Count} warning(s) were generated during code generation. Please check the console for details.",
                                ok: "OK");
                        }
                    }
                    catch (MessageTokenizerException e)
                    {
                        Debug.LogError(e.ToString() + e.Message);
                        EditorUtility.DisplayDialog(
                            title: "Message Tokenizer Error",
                            message: $"A tokenizer error occurred while processing the file:\n{inFilePath}\n\n{e.Message}",
                            ok: "OK");
                    }
                    catch (MessageParserException e)
                    {
                        Debug.LogError(e.ToString() + e.Message);
                        EditorUtility.DisplayDialog(
                            title: "Message Parser Error",
                            message: $"A parser error occurred while processing the file:\n{inFilePath}\n\n{e.Message}",
                            ok: "OK");
                    }
                }
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        protected abstract List<string> Generate(string inPath, string outPath, bool isRos2, string rosPackageName = "");
    }
}
