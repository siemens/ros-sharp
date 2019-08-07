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

using UnityEngine;

using RosSharp.RosBridgeClient.Actionlib;
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
        public int timeout = 10;
        public float timeStep = 0.2f;
        public int fibonacciOrder = 20;

        private FibonacciActionClient client;

        public string status = "";
        public string feedback = "";
        public string result = "";

        private void Awake()
        {
            FibonacciAction action = new FibonacciAction();
            action.action_goal.goal.order = fibonacciOrder;
            client = new FibonacciActionClient(action, actionName, serverURL, protocol, serializer, timeout, timeStep);
        }

        // Start is called before the first frame update
        private void Start()
        {
            client.Start();
        }

        // Update is called once per frame
        private void Update()
        {
            status = client.GetStatusString();
            feedback = client.GetFeedbackString();
            result = client.GetResultString();
        }

        private void OnDestroy()
        {
            client.Stop();
        }

        public void SendGoal()
        {
            client.SendGoalFromUnity();
        }

        public void CancelGoal()
        {
            client.CancelGoalFromUnity();
        }
    }

    public class FibonacciActionClient : ActionClient<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public FibonacciActionClient(FibonacciAction action, string actionName, string serverURL, Protocol protocol, RosSocket.SerializerEnum serializer, int timeout, float timeStep) : base(action, actionName, serverURL, protocol, serializer, timeStep) { }

        protected override string GoalID()
        {
            return "fibonacci-unity-" + Guid.NewGuid();
        }

        protected override void OnFeedbackReceived()
        {
            // Not implemented since get string directly returns stored feedback
        }

        protected override void OnResultReceived()
        {
            // Not implemented since get string directly returns stored result
        }

        public string GetStatusString()
        {
            if (goalStatus != null) {
                return ((ActionStatus)(goalStatus.status)).ToString();
            }
            return "";
        }

        public string GetFeedbackString()
        {
            return String.Join(",", action.action_feedback.feedback.sequence);
        }

        public string GetResultString()
        {
            return String.Join(",", action.action_result.result.sequence);
        }

        public void SendGoalFromUnity()
        {
            SendGoal();
        }

        public void CancelGoalFromUnity()
        {
            CancelGoal();
        }

        protected override void Log(string log)
        {
            Debug.Log("Fibonacci Action Client: " + log);
        }

        protected override void LogWarning(string log)
        {
            Debug.LogWarning("Fibonacci Action Client: " + log);
        }

        protected override void LogError(string log)
        {
            Debug.LogError("Fibonacci Action Client: " + log);
        }
    }
}

