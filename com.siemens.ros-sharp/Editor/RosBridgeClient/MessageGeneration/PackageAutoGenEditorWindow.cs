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
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public abstract class PackageAutoGenEditorWindow : EditorWindow
    {
        [SerializeField]
        private static string lastPackageDirectory = string.Empty;
        [SerializeField]
        private static string lastOutputDirectory = string.Empty;

        private string inPkgPath = "";
        private string rosPackageName = "";
        protected bool toggleROS2 = true;
        private string outPkgPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosSharpMessages");

        protected abstract string GenerationType { get; }
        protected abstract string FileExtension { get; }

        protected virtual void OnGUI()
        {
            GUILayout.Label("Package " + GenerationType + " auto generation", EditorStyles.boldLabel);

            toggleROS2 = GUILayout.Toggle(toggleROS2, "ROS2 Message");
            
            EditorGUILayout.BeginHorizontal();
            inPkgPath = EditorGUILayout.TextField("Input Package Path", inPkgPath);
            
            if (GUILayout.Button("Browse Package...", GUILayout.Width(150)))
            {
                inPkgPath = EditorUtility.OpenFolderPanel("Select Package...", lastPackageDirectory, "");
                if (!inPkgPath.Equals(""))
                {
                    rosPackageName = inPkgPath.Split('/').Last();
                }
            }
            EditorGUILayout.EndHorizontal();

            rosPackageName = EditorGUILayout.TextField("ROS Package Name:", rosPackageName);

            EditorGUILayout.BeginHorizontal();
            outPkgPath = EditorGUILayout.TextField("Output Location", outPkgPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                outPkgPath = EditorUtility.OpenFolderPanel("Select Folder...", lastOutputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("Generate"))
            {
                if (inPkgPath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Input Package Path Required",
                        message: "The input package path is empty. Please specify a valid package directory.",
                        ok: "OK");
                }
                else
                {
                    lastPackageDirectory = inPkgPath;
                    lastOutputDirectory = outPkgPath;
                    try
                    {
                        string[] files = Directory.GetFiles(Path.Combine(inPkgPath, FileExtension), "*." + FileExtension);
                        if (files.Length == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: $"No {GenerationType} Files Found",
                                message: $"No {GenerationType} files were found in the specified package directory.",
                                ok: "OK");
                            Reset();
                        }
                        else
                        {
                            // Keep a list of warnings
                            List<string> warnings = new List<string>();
                            for (int i = 0; i < files.Length; i++)
                            {
                                string file = files[i];
                                EditorUtility.DisplayProgressBar(
                                    $"Processing {GenerationType} Files ({i + 1}/{files.Length})",
                                    $"Parsing: {file}",
                                    (i + 1) / (float)files.Length);
                                try
                                {
                                    warnings.AddRange(Generate(file, outPkgPath, toggleROS2, rosPackageName));
                                }
                                catch (MessageTokenizerException e)
                                {
                                    Debug.LogError(e.ToString() + e.Message);
                                    EditorUtility.DisplayDialog(
                                        title: "Message Tokenizer Error",
                                        message: $"A tokenizer error occurred while processing the file:\n{file}\n\n{e.Message}",
                                        ok: "OK");
                                }
                                catch (MessageParserException e)
                                {
                                    Debug.LogError(e.ToString() + e.Message);
                                    EditorUtility.DisplayDialog(
                                        title: "Message Parser Error",
                                        message: $"A parser error occurred while processing the file:\n{file}\n\n{e.Message}",
                                        ok: "OK");
                                }
                            }
                            // Done
                            EditorUtility.ClearProgressBar();
                            AssetDatabase.Refresh();
                            if (warnings.Count > 0)
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Completed with Warnings",
                                    message: $"Output location: {outPkgPath}\n{warnings.Count} warning(s) were generated during code generation. Please check the console for details.",
                                    ok: "OK");
                                foreach (string w in warnings)
                                {
                                    Debug.LogWarning(w);
                                }
                            }
                            else
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Successful",
                                    message: $"Code generation completed successfully. Output location: {outPkgPath}",
                                    ok: "OK");
                            }
                            Reset();
                        }
                    }
                    catch (DirectoryNotFoundException e)
                    {
                        EditorUtility.DisplayDialog(
                            title: "Directory Not Found",
                            message: $"The specified directory could not be found.\n\n{e.Message}",
                            ok: "OK");
                        Reset();
                    }
                }
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void Reset()
        {
            inPkgPath = "";
            rosPackageName = "";
            outPkgPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosSharpMessages");
        }

        protected abstract List<string> Generate(string inPath, string outPath, bool isRos2, string rosPackageName = "");
    }
}
