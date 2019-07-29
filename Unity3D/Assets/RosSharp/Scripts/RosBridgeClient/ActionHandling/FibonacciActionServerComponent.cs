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

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;


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
            feedback = server.GetFeedbackString();
        }

        private void OnDestroy()
        {
            server.Stop();
        }
    }

    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);

        public FibonacciActionServer(FibonacciAction action, string actionName, Protocol protocol, string serverURL, RosSocket.SerializerEnum serializer, int timeout, float timeStep) : base(action, actionName, protocol, serverURL, serializer, timeout, timeStep) { }

        protected override bool IsGoalValid()
        {
            if (action.action_goal.goal.order < 1)
            {
                Debug.LogWarning("Cannot generate fibonacci sequence of order less than 1");
                return false;
            }
            return true;
        }

        protected override void WaitForGoal()
        {
            // We don't wait for goal in this example,
            // since Unity monobehaviour will be spinning in play mode.
        }

        protected override void GoalHandler()
        {
            isProcessingGoal.Set();

            Debug.Log("Generating Fibonacci sequence of order " + action.action_goal.goal.order + " with seeds 0, 1");

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                if (!isProcessingGoal.WaitOne(0))
                {
                    return;
                }
                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();

                Thread.Sleep(millisecondsTimestep);
            }

            UpdateAndPublishStatus(ActionStatus.SUCCEEDED);
            action.action_result.result.sequence = sequence.ToArray();
            PublishResult();
            Debug.Log("Result Published to client...");
            isProcessingGoal.Reset();

            Thread.Sleep(millisecondsTimestep);
            Debug.Log("Ready for next goal...(status = PENDING)");
            action.action_feedback = new FibonacciActionFeedback();
            UpdateAndPublishStatus(ActionStatus.PENDING);
        }

        protected override void CancellationHandler()
        {
            isProcessingGoal.Reset();
            Debug.Log("Goal cancelled by client");
        }

        public void UpdateStatus()
        {
            PublishStatus();
        }

        public string GetStatusString()
        {
            return actionStatus.ToString();
        }

        public string GetFeedbackString()
        {
            return String.Join(",", action.action_feedback.feedback.sequence);
        }
    }
}



