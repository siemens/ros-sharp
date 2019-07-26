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

using System.Collections.Generic;

using System.Linq;

using UnityEngine;
using UnityEditor;

using RosSharp.RosBridgeClient.MessageGeneration;

namespace RosSharp.RosBridgeClient
{
    public class PackageMsgAutoGenEditorWindow : EditorWindow {
        private string inPkgPath = "";
        private string rosPackageName = "";
        private string outPkgPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosSharpMessages");

        [MenuItem("RosBridgeClient/Auto Generate Messages/Package Messages...", false, 1)]
        private static void OpenWindow() {
            PackageMsgAutoGenEditorWindow window = GetWindow<PackageMsgAutoGenEditorWindow>(false, "Message Auto Generation", true);
            window.minSize = new Vector2(750, 100);
            window.maxSize = new Vector2(750, 100);
            window.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("Package messages auto generation", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            inPkgPath = EditorGUILayout.TextField("Input Package Path", inPkgPath);
            if (GUILayout.Button("Browse Package...", GUILayout.Width(150)))
            {
                inPkgPath = EditorUtility.OpenFolderPanel("Select Package...", "", "");
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
                outPkgPath = EditorUtility.OpenFolderPanel("Select Folder...", "", "");
            }
            EditorGUILayout.EndHorizontal();
            
            if (GUILayout.Button("GENERATE!"))
            {
                if (inPkgPath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Error", 
                        message: "Empty input package path!\nPlease specify input package", 
                        ok: "Bricks without straw");
                }
                else
                {
                    try {
                        string[] files = Directory.GetFiles(Path.Combine(inPkgPath, "msg"), "*.msg");
                        if (files.Length == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: "No message files found!", 
                                message: "No message files found!", 
                                ok: "Bricks without straw");
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
                                    "Working...(" + (i+1) + "/" + files.Length + ")", 
                                    "Parsing " + file, 
                                    (float)(i+1)/(float)files.Length);
                                try
                                {
                                    warnings.AddRange(MessageAutoGen.GenerateSingleMessage(file, outPkgPath, rosPackageName));
                                }
                                catch (MessageTokenizerException e)
                                {
                                    Debug.LogError(e.ToString() + e.Message);
                                    EditorUtility.DisplayDialog(
                                        title: "Message Tokenizer Exception", 
                                        message: e.Message, 
                                        ok: "Wait. That's illegal");
                                }
                                catch (MessageParserException e)
                                {
                                    Debug.LogError(e.ToString() + e.Message);
                                    EditorUtility.DisplayDialog(
                                        title: "Message Parser Exception", 
                                        message: e.Message, 
                                        ok: "Sorry but you can't ignore errors.");
                                }
                            }
                            // Done
                            EditorUtility.ClearProgressBar();
                            if (warnings.Count > 0)
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Complete", 
                                    message: "Output at: " + outPkgPath + "\nYou have " + warnings.Count + " warning(s)", 
                                    ok: "I like to live dangerously");
                                foreach (string w in warnings)
                                {
                                    Debug.LogWarning(w);
                                }
                            }
                            else
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Complete", 
                                    message: "Output at: " + outPkgPath, 
                                    ok: "Thank you!");
                            }
                            Reset();
                        }
                    }
                    catch (DirectoryNotFoundException e) {
                        EditorUtility.DisplayDialog(
                            title: "Message Folder not found", 
                            message: e.Message, 
                            ok: "Bricks without straw");
                        Reset();
                    }
                }
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void Reset() {
            inPkgPath = "";
            rosPackageName = "";
            outPkgPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosSharpMessages");
        }
    }
}