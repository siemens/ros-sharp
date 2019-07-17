﻿/*
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

using System;

using UnityEditor;
using UnityEngine;

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;

namespace RosSharp.RosBridgeClient
{
    public class FibonacciActionClientComponent : MonoBehaviour
    {
        public string actionName = "fibonacci";
        public Protocol protocol = Protocol.WebSocketSharp;
        public string serverURL = "ws://192.168.137.195:9090";
        public RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON;
        public bool serverIsUp = false;
        public int timeout = 10;
        public float timeStep = 0.2f;
        public int fibonacciOrder = 10;

        private FibonacciActionClient client;

        // Start is called before the first frame update
        private void Start()
        {
            FibonacciAction action = new FibonacciAction();
            action.action_goal.goal.order = fibonacciOrder;
            client = new FibonacciActionClient(action, actionName, protocol, serverURL, serializer, timeout, timeStep);
        }

        // Update is called once per frame
        private void Update()
        {
            serverIsUp = client.IsSeverUp();
        }

        private void OnDestroy()
        {
            client.Stop();
        }

        public void SendGoal() {
            client.SendGoal();
        }

        public void CancelGoal() {
            client.CancelGoal();
        }

        public string GetStatus()
        {
            if (client == null) {
                return "";
            }
            return client.GetStatus();
        }

        public string GetFeedback() {
            if (client == null)
            {
                return "";
            }
            return client.GetFeedback();
        }

        public string GetResult() {
            if (client == null)
            {
                return "";
            }
            return client.GetResult();
        }
    }

    public class FibonacciActionClient : ActionClient<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public FibonacciActionClient(FibonacciAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer, int timeout, float timeStep) : base(action, actionName, protocol, serverURL, serializer, timeout, timeStep) { }

        private string feedback = "";
        private string result = "";

        protected override void FeedbackHandler()
        {
            feedback = String.Join(",", action.action_feedback.feedback.sequence);
        }

        protected override void ResultHandler()
        {
            result = String.Join(",", action.action_result.result.sequence);
        }

        public bool IsSeverUp() {
            return isServerUp;
        }

        public string GetStatus()
        {
            return actionStatus.ToString();
        }

        public string GetFeedback() {
            return feedback;
        }

        public string GetResult() {
            return result;
        }
    }

    [CustomEditor(typeof(FibonacciActionClientComponent))]
    public class FibonacciActionClientEditor : Editor {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            if (GUILayout.Button("Send Goal")) {
                ((FibonacciActionClientComponent)target).SendGoal();
            }

            if (GUILayout.Button("Cancel Goal")) {
                ((FibonacciActionClientComponent)target).CancelGoal();
            }

            EditorGUILayout.TextField("Status: ", ((FibonacciActionClientComponent)target).GetStatus());
            EditorGUILayout.TextField("Feedback: ", ((FibonacciActionClientComponent)target).GetFeedback());
            EditorGUILayout.TextField("Result: ", ((FibonacciActionClientComponent)target).GetResult());

            Repaint();
        }
    }
}

