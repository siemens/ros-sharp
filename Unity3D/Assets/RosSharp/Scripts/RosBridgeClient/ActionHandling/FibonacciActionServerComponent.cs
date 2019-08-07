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

using System;
using System.Threading;
using System.Collections.Generic;

using UnityEngine;

using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public class FibonacciActionServerComponent : MonoBehaviour
    {
        public string actionName = "fibonacci";
        public Protocol protocol = Protocol.WebSocketSharp;
        public string serverURL = "ws://192.168.137.195:9090";
        public RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON;
        public int timeout = 10;
        public float timeStep = 0.2f;

        private FibonacciActionServer server;

        public string status = "";
        public string feedback = "";

        private void Awake()
        {
            server = new FibonacciActionServer(new FibonacciAction(), actionName, protocol, serverURL, serializer, timeout, timeStep);
        }

        // Start is called before the first frame update
        private void Start()
        {
            server.Start();
        }

        // Update is called once per frame
        private void Update()
        {
            server.UpdateStatus();
            status = server.GetStatusString();
            feedback = server.GetFeedbackSequenceString();
        }

        private void OnDestroy()
        {
            server.Stop();
        }
    }

    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);

        private Thread goalHandler;

        public FibonacciActionServer(FibonacciAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer, int timeout, float timeStep) : base(action, actionName, protocol, serverURL, serializer, timeout, timeStep) { }


        protected bool IsGoalValid()
        {
            return action.action_goal.goal.order >= 1;
        }

        private void ExecuteFibonacciGoal()
        {
            isProcessingGoal.Set();

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                if (!isProcessingGoal.WaitOne(0))
                {
                    action.action_result.result.sequence = sequence.ToArray();
                    SetCanceled();
                    return;
                }

                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();

                Thread.Sleep(millisecondsTimestep);
            }

            action.action_result.result.sequence = sequence.ToArray();
            SetSucceeded();
        }

        public void UpdateStatus()
        {
            PublishStatus();
        }

        public string GetStatusString()
        {
            return GetStatus().ToString();
        }

        public string GetFeedbackSequenceString()
        {
            return String.Join(",", action.action_feedback.feedback.sequence);
        }


        protected override void OnGoalReceived()
        {
            if (IsGoalValid())
            {
                SetAccepted("Fibonacci Action Server: The goal has been accepted");
            }
            else
            {
                SetRejected("Fibonacci Action Server: Cannot generate fibonacci sequence of order less than 1");
            }
        }

        protected override void OnGoalRecalling(GoalID goalID)
        {
            // Left blank for this example
        }

        protected override void OnGoalRejected()
        {
            LogWarning("Cannot generate fibonacci sequence of order less than 1. Goal Rejected");
        }

        protected override void OnGoalActive()
        {
            goalHandler = new Thread(ExecuteFibonacciGoal);
            goalHandler.Start();
        }

        protected override void OnGoalPreempting()
        {
            isProcessingGoal.Reset();
            goalHandler.Join();
        }

        protected override void OnGoalSucceeded()
        {
            isProcessingGoal.Reset();
            Thread.Sleep(millisecondsTimestep);
            UpdateAndPublishStatus(ActionStatus.NO_GOAL);
        }

        protected override void OnGoalAborted()
        {
            // Left blank for this example
        }

        protected override void OnGoalCanceled()
        {
            PublishResult();
        }

        protected override void Log(string log)
        {
            Debug.Log("Fibonacci Action Server: " + log);
        }

        protected override void LogWarning(string log)
        {
            Debug.LogWarning("Fibonacci Action Server: " + log);
        }

        protected override void LogError(string log)
        {
            Debug.LogError("Fibonacci Action Client: " + log);
        }
    }
}



