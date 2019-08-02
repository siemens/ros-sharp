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
            Console.WriteLine("Waiting for goal");
            Thread waitForGoal = new Thread(WaitForGoal);
            waitForGoal.Start();

            Console.WriteLine("Press any key to stop server...\n");
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

            Console.WriteLine("Generating Fibonacci sequence of order " + action.action_goal.goal.order + " with seeds 0, 1");

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                if (!isProcessingGoal.WaitOne(0))
                {
                    action.action_result.result.sequence = sequence.ToArray();
                    SetCanceled();
                    Console.WriteLine("Press any key to stop server...\n");
                    return;
                }

                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();
                Console.WriteLine(GetFeedbackLogString());

                Thread.Sleep(millisecondsTimestep);
            }

            action.action_result.result.sequence = sequence.ToArray();
            SetSucceeded();
        }

        protected override void OnGoalRecalling(GoalID goalID)
        {
            // Left blank for this example
        }

        protected override void OnGoalPreempting()
        {
            isProcessingGoal.Reset();
            Console.WriteLine("Preempting goal");
            goalHandle.Join();
        }

        protected override void OnGoalReceived()
        {
            if (IsGoalValid())
            {
                SetAccepted("The goal has been accepted by RosSharp Fibonacci Action Console Server");
            }
            else
            {
                SetRejected("Cannot generate fibonacci sequence of order less than 1");
            }
        }

        protected override void OnGoalActive()
        {
            goalHandle = new Thread(ExecuteFibonacciGoal);
            goalHandle.Start();
        }

        protected override void OnGoalRejected()
        {
            LogWarning("Cannot generate fibonacci sequence of order less than 1. Goal Rejected");
        }

        protected override void OnGoalSucceeded()
        {
            Console.WriteLine(GetResultLogString());
            Console.WriteLine("Result Published to client...");

            isProcessingGoal.Reset();
            Thread.Sleep(millisecondsTimestep);
            Console.WriteLine("Ready for next goal...");
            UpdateAndPublishStatus(ActionStatus.NO_GOAL);
            Console.WriteLine("Press any key to stop server...\n");
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
            Console.WriteLine("Fibonacci Action Console Server: [LOG] " + log);
        }

        protected override void LogWarning(string log)
        {
            Console.WriteLine("Fibonacci Action Console Server: [WARNING] " + log);
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
