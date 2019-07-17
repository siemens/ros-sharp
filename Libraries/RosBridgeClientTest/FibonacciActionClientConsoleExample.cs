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

namespace RosSharp.RosBridgeClientTest
{
    public class FibonacciActionConsoleClient : ActionClient<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public FibonacciActionConsoleClient(FibonacciAction action, string actionName, Protocol protocol, string serverURL) : base(action, actionName, protocol, serverURL) { }

        public void Execute() {
            Console.WriteLine("Waiting for server...\n");
            WaitForServer();
            Console.WriteLine("Send goal...\n");
            SendGoal();
        }

        protected override void FeedbackHandler()
        {
            Console.WriteLine("Feedback @ " + DateTime.Now);
            Console.WriteLine("Server status: " + (ActionStatus)action.action_feedback.status.status);
            Console.WriteLine("Sequence: ");
            PrintIntArray(action.action_feedback.feedback.sequence);
            Console.WriteLine("---");
        }

        protected override void ResultHandler()
        {
            Console.WriteLine("Result @ " + DateTime.Now);
            Console.WriteLine("Server status: " + (ActionStatus)action.action_result.status.status);
            Console.WriteLine("Sequence: ");
            PrintIntArray(action.action_result.result.sequence);
            StopUpdateServerStatus();
        }

        private void PrintIntArray(int[] array) {
            Console.WriteLine("[{0}]", string.Join(", ", array));
        }
    }

    public class FibonacciActionClientConsoleExample
    {
        public static void Main(string[] args) {
            FibonacciAction action = new FibonacciAction();
            action.action_goal.goal.order = 20;
            FibonacciActionConsoleClient client = new FibonacciActionConsoleClient(action, "fibonacci", Protocol.WebSocketSharp, "ws://192.168.137.195:9090");
            client.Execute();
        }
    }
}