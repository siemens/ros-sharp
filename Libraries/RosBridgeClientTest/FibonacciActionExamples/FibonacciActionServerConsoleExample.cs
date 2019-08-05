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
using System.Collections.Generic;
using System.Threading;

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClientTest
{
    public class FibonacciActionConsoleServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        private ManualResetEvent isWaitingForGoal = new ManualResetEvent(false);
        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);

        private Thread goalHandle;

        public FibonacciActionConsoleServer(FibonacciAction action, string actionName, Protocol protocol, string serverURL) : base(action, actionName, protocol, serverURL) { }

        public void Execute()
        {
            Start();

            isWaitingForGoal.Set();
            Log("Waiting for goal...");
            Thread waitForGoal = new Thread(WaitForGoal);
            waitForGoal.Start();

            Log("Press any key to stop server...\n");
            Console.ReadKey(true);
            isWaitingForGoal.Reset();
            waitForGoal.Join();
            Stop();
        }

        protected void WaitForGoal()
        {
            while (isWaitingForGoal.WaitOne(0))
            {
                PublishStatus();
                Thread.Sleep(millisecondsTimestep);
            }
        }

        protected bool IsGoalValid()
        {
            return action.action_goal.goal.order >= 1; 
        }

        private void ExecuteFibonacciGoal()
        {
            isProcessingGoal.Set();

            Log("Generating Fibonacci sequence of order " + action.action_goal.goal.order + " with seeds 0, 1");

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                if (!isProcessingGoal.WaitOne(0))
                {
                    action.action_result.result.sequence = sequence.ToArray();
                    SetCanceled();
                    Log("Press any key to stop server...\n");
                    return;
                }

                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();
                Log(GetFeedbackLogString());

                Thread.Sleep(millisecondsTimestep);
            }

            action.action_result.result.sequence = sequence.ToArray();
            SetSucceeded();
        }


        protected override void OnGoalReceived()
        {
            if (IsGoalValid())
            {
                SetAccepted("Fibonacci Action Server: The goal has been accepted by RosSharp Fibonacci Action Console Server");
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
            goalHandle = new Thread(ExecuteFibonacciGoal);
            goalHandle.Start();
        }

        protected override void OnGoalPreempting()
        {
            isProcessingGoal.Reset();
            Log("Preempting goal");
            goalHandle.Join();
        }

        protected override void OnGoalSucceeded()
        {
            Log(GetResultLogString());
            Log("Result Published to client...");

            isProcessingGoal.Reset();
            Thread.Sleep(millisecondsTimestep);
            Log("Ready for next goal...");
            UpdateAndPublishStatus(ActionStatus.NO_GOAL);
            Log("Press any key to stop server...\n");
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
            Console.WriteLine("Fibonacci Action Server @ " + DateTime.Now + " : [LOG] " + log);
        }

        protected override void LogWarning(string log)
        {
            Console.WriteLine("Fibonacci Action Server @ " + DateTime.Now + " : [WARNING] " + log);
        }

        protected override void LogError(string log)
        {
            Console.Error.WriteLine("Fibonacci Action Server @ " + DateTime.Now + " : [ERROR] " + log);
        }
    }

    public class FibonacciActionServerConsoleExample
    {
        public static void Main(string[] args) {
            FibonacciActionConsoleServer server = new FibonacciActionConsoleServer(new FibonacciAction(), "fibonacci", Protocol.WebSocketSharp, "ws://192.168.137.195:9090");
            server.Execute();
        }
    }
}
