/*
© Siemens AG, 2019
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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
using UnityEngine;
using UnityEditor;
using RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClient
{
    public class SimpleMessageGenerationEditorWindow : EditorWindow
    {
        private static string assetPath;
        private static string messageName;
        private static string rosPackageName;
        public MessageElement[] messageElements;

        private SerializedObject serializedObject;
        private SerializedProperty serializedProperty;

        [MenuItem("RosBridgeClient/Generate Messages/Simple Messages")]
        private static void Init()
        {
            SimpleMessageGenerationEditorWindow editorWindow = GetWindow<SimpleMessageGenerationEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);
            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("Custom Message Generator (Simple Message)", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            messageName = EditorGUILayout.TextField("Message Name", messageName);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            rosPackageName = EditorGUILayout.TextField("ROS Package Name", rosPackageName);
            EditorGUILayout.EndHorizontal();


            EditorGUILayout.BeginHorizontal();
            ScriptableObject target = this;
            serializedObject = new SerializedObject(target);
            serializedObject.Update();
            serializedProperty = serializedObject.FindProperty("messageElements");
            EditorGUILayout.PropertyField(serializedProperty, true);
            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.EndHorizontal();
            

            GUILayout.Space(40);
            EditorGUILayout.BeginHorizontal();
            assetPath = EditorGUILayout.TextField("Asset Path", assetPath);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.Space();
            if (GUILayout.Button("Set Default Path", GUILayout.Width(150)))
            {
                DeleteEditorPrefs();
                GetEditorPrefs();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Generate Simple Message"))
            {
                SetEditorPrefs();
                Debug.Log("generate button is pressed in editor");
                SimpleMessageGenerator.Generate(messageName, rosPackageName, messageElements, assetPath);
                AssetDatabase.Refresh();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);

            EditorGUIUtility.labelWidth = 300;

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
        }

        private void DeleteEditorPrefs()
        {
            EditorPrefs.DeleteKey("AssetPath");
        }

        private void GetEditorPrefs()
        {
            assetPath = (EditorPrefs.HasKey("AssetPath") ?
                        EditorPrefs.GetString("AssetPath") :
                        Path.Combine(Path.GetFullPath("."), "Assets"));
        }

        private void SetEditorPrefs()
        {
            EditorPrefs.SetString("AssetPath", assetPath);
            EditorPrefs.SetString("MessageName", messageName);
            EditorPrefs.SetString("RosPackageName", rosPackageName);
        }

        #endregion
    }
}
