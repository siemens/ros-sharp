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
using MessageType = RosBridgeClient.Messages.MessageType;

namespace RosSharp.RosBridgeClient
{
    public class ServiceMessageGenerationEditorWindow : EditorWindow
    {
        private static string assetPath;
        private static string serviceName;
        private static string rosPackageName;
        public MessageElement[] requestElements;
        public MessageElement[] responseElements;

        private SerializedObject    serializedObject_request;
        private SerializedProperty  serializedProperty_request;
        private SerializedObject    serializedObject_response;
        private SerializedProperty  serializedProperty_response;


        [MenuItem("RosBridgeClient/Generate Messages/Service Messages")]
        private static void Init()
        {
            ServiceMessageGenerationEditorWindow editorWindow = GetWindow<ServiceMessageGenerationEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);
            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("Message Generator (Service Message)", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            serviceName = EditorGUILayout.TextField("Service Name", serviceName);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            rosPackageName = EditorGUILayout.TextField("ROS Package Name", rosPackageName);
            EditorGUILayout.EndHorizontal();


            EditorGUILayout.BeginHorizontal();
            ScriptableObject target_request = this;
            serializedObject_request = new SerializedObject(target_request);
            serializedObject_request.Update();
            serializedProperty_request = serializedObject_request.FindProperty("requestElements");
            EditorGUILayout.PropertyField(serializedProperty_request, true);
            serializedObject_request.ApplyModifiedProperties();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            ScriptableObject target = this;
            serializedObject_response = new SerializedObject(target);
            serializedObject_response.Update();
            serializedProperty_response = serializedObject_response.FindProperty("responseElements");
            EditorGUILayout.PropertyField(serializedProperty_response, true);
            serializedObject_response.ApplyModifiedProperties();
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(40);
            EditorGUILayout.BeginHorizontal();
            assetPath = EditorGUILayout.TextField("Asset Path", assetPath);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.Space();
            if (GUILayout.Button("Set Default", GUILayout.Width(150)))
            {
                DeleteEditorPrefs();
                GetEditorPrefs();
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(20);
            EditorGUILayout.BeginHorizontal();

            if (GUILayout.Button("Generate Service Message"))
            {
                SetEditorPrefs();
                ServiceMessageGenerator.Generate(serviceName, rosPackageName, requestElements, responseElements, assetPath);
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
            assetPath       = (EditorPrefs.HasKey("AssetPath") ?
                              EditorPrefs.GetString("AssetPath") :
                              Path.Combine(Path.GetFullPath("."), "Assets"));

            serviceName     = (EditorPrefs.HasKey("ServiceName") ?
                              EditorPrefs.GetString("ServiceName") :
                              "AddTwoInts");

            rosPackageName = (EditorPrefs.HasKey("ROSPackageName") ?
                              EditorPrefs.GetString("ROSPackageName") :
                              "beginner_tutorials");

            requestElements = new MessageElement[] { new MessageElement { messageType = MessageType.@int, messageName = "a" }, new MessageElement { messageType = MessageType.@int, messageName = "b" } };
            responseElements = new MessageElement[] { new MessageElement { messageType = MessageType.@int, messageName = "sum" } };
        }

        private void SetEditorPrefs()
        {
            EditorPrefs.SetString("AssetPath", assetPath);
            EditorPrefs.SetString("MessageName", serviceName);
            EditorPrefs.SetString("RosPackageName", rosPackageName);
        }

        #endregion
    }
}
