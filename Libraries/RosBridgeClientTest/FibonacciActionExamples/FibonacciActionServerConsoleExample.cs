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

namespace RosSharp.RosBridgeClientTest
{
    public class FibonacciActionConsoleServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        private ManualResetEvent isWaitingForGoal = new ManualResetEvent(false);
        private ManualResetEvent isProcessingGoal = new ManualResetEvent(false);

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

        protected override void WaitForGoal()
        {
            while (isWaitingForGoal.WaitOne(0))
            {
                PublishStatus();
                Thread.Sleep(millisecondsTimestep);
            }
        }

        protected override bool IsGoalValid()
        {
            if (action.action_goal.goal.order < 1) {
                Console.WriteLine("Cannot generate fibonacci sequence of order less than 1");
                return false;
            }
            return true;
        }

        protected override void GoalHandler()
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
                    Console.WriteLine("Goal cancelled by client");
                    return;
                }
                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();
                Console.WriteLine(GetFeedbackLogString());

                Thread.Sleep(millisecondsTimestep);
            }

            UpdateAndPublishStatus(ActionStatus.SUCCEEDED);
            action.action_result.result.sequence = sequence.ToArray();
            PublishResult();
            Console.WriteLine(GetResultLogString());
            Console.WriteLine("Result Published to client...");
            isProcessingGoal.Reset();

            Thread.Sleep(millisecondsTimestep);
            Console.WriteLine("Ready for next goal...(status = PENDING)");
            action.action_feedback = new FibonacciActionFeedback();
            UpdateAndPublishStatus(ActionStatus.PENDING);
            Console.WriteLine("Press any key to stop server...\n");
        }

        protected override void CancellationHandler()
        {
            isProcessingGoal.Reset();
            Console.WriteLine("Goal cancelled by client");
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
