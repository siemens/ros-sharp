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

using UnityEngine;
using UnityEditor;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public class SingleMsgAutoGenEditorWindow : EditorWindow
    {
        private string inFilePath = "";
        private string outFilePath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosSharpMessages");
        private string rosPackageName = "";

        [MenuItem("RosBridgeClient/Auto Generate Messages/Single Message...", false, 0)]
        private static void OpenWindow() {
            SingleMsgAutoGenEditorWindow window = GetWindow<SingleMsgAutoGenEditorWindow>(false, "Message Auto Generation", true);
            window.minSize = new Vector2(750, 100);
            window.maxSize = new Vector2(750, 100);
            window.Show();
        }

        private void OnGUI() {
            GUILayout.Label("Single message auto generation", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            inFilePath = EditorGUILayout.TextField("Input File Path", inFilePath);
            if (GUILayout.Button("Browse File...", GUILayout.Width(120))) {
                inFilePath = EditorUtility.OpenFilePanel("Select Message File...", "", "msg");
                if (!inFilePath.Equals("")) {
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
                outFilePath = EditorUtility.OpenFolderPanel("Select Folder...", "", "");
            }
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("GENERATE!")) {
                if (inFilePath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Error", 
                        message: "Empty input file path!\nPlease specify input file", 
                        ok: "Bricks without straw");
                }
                else
                {
                    try
                    {
                        List<string> warnings = MessageAutoGen.GenerateSingleMessage(inFilePath, outFilePath, rosPackageName);
                        if (warnings.Count == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: "Code Generation Complete", 
                                message: "Output at: " + outFilePath, 
                                ok: "Thank you!");
                        }
                        else {
                            foreach (string w in warnings) {
                                Debug.LogWarning(w);
                            }
                            EditorUtility.DisplayDialog(
                                title: "Code Generation Complete", 
                                message: "Output at: " + outFilePath + "\nYou have " + warnings.Count + " warning(s)", 
                                ok: "I like to live dangerously");
                        }
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
            }       
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }
    }
}