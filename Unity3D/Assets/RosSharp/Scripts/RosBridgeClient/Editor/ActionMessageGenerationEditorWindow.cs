﻿/*
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
    public class ActionMessageGenerationEditorWindow : EditorWindow
    {
        private static string assetPath;
        private static string actionName;
        private static string rosPackageName;
        public MessageElement[] goalElements;
        public MessageElement[] resultElements;
        public MessageElement[] feedbackElements;

        private SerializedObject    serializedObject_goal;
        private SerializedProperty  serializedProperty_goal;
        private SerializedObject    serializedObject_result;
        private SerializedProperty  serializedProperty_result;
        private SerializedObject    serializedObject_feedback;
        private SerializedProperty  serializedProperty_feedback;

        [MenuItem("RosBridgeClient/Generate Messages/Action Messages")]
        private static void Init()
        {
            ActionMessageGenerationEditorWindow editorWindow = GetWindow<ActionMessageGenerationEditorWindow>();
            editorWindow.minSize = new Vector2(500, 300);
            editorWindow.Show();
        }

        private void OnGUI()
        {
            GUILayout.Label("Message Generator (Action Message)", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            actionName = EditorGUILayout.TextField("Action Name", actionName);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            rosPackageName = EditorGUILayout.TextField("ROS Package Name", rosPackageName);
            EditorGUILayout.EndHorizontal();


            EditorGUILayout.BeginHorizontal();
            ScriptableObject target_goal = this;
            serializedObject_goal = new SerializedObject(target_goal);
            serializedObject_goal.Update();
            serializedProperty_goal = serializedObject_goal.FindProperty("goalElements");
            EditorGUILayout.PropertyField(serializedProperty_goal, true);
            serializedObject_goal.ApplyModifiedProperties();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            ScriptableObject target = this;
            serializedObject_result = new SerializedObject(target);
            serializedObject_result.Update();
            serializedProperty_result = serializedObject_result.FindProperty("resultElements");
            EditorGUILayout.PropertyField(serializedProperty_result, true);
            serializedObject_result.ApplyModifiedProperties();
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            ScriptableObject target_feedback = this;
            serializedObject_feedback = new SerializedObject(target);
            serializedObject_feedback.Update();
            serializedProperty_feedback = serializedObject_feedback.FindProperty("feedbackElements");
            EditorGUILayout.PropertyField(serializedProperty_feedback, true);
            serializedObject_feedback.ApplyModifiedProperties();
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

            if (GUILayout.Button("Generate Action Messages"))
            {
                SetEditorPrefs();
                ActionMessageGenerator.Generate(actionName, rosPackageName, 
                                                goalElements, resultElements, feedbackElements, assetPath);
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

            actionName = (EditorPrefs.HasKey("ActionName") ?
                          EditorPrefs.GetString("ActionName") :
                          "Fibonacci");

            rosPackageName = (EditorPrefs.HasKey("ROSPackageName") ?
                             EditorPrefs.GetString("ROSPackageName") :
                             "actionlib_tutorials");

            goalElements     = new MessageElement[] { new MessageElement { messageType = MessageType.@int, messageName = "order" } };
            resultElements   = new MessageElement[] { new MessageElement { messageType = MessageType.@int, messageName = "sequence", isArray = true } };
            feedbackElements = new MessageElement[] { new MessageElement { messageType = MessageType.@int, messageName = "sequence", isArray = true } };
        }

        private void SetEditorPrefs()
        {
            EditorPrefs.SetString("AssetPath", assetPath);
            EditorPrefs.SetString("MessageName", actionName);
            EditorPrefs.SetString("RosPackageName", rosPackageName);
        }

        #endregion
    }
}
