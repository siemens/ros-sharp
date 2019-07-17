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
        public FibonacciActionConsoleServer(FibonacciAction action, string actionName, Protocol protocol, string serverURL) : base(action, actionName, protocol, serverURL) { }

        protected override bool IsGoalValid()
        {
            if (action.action_goal.goal.order <= 0) {
                Console.WriteLine("Cannot generate fibonacci sequence of order less than 1");
            }
            return action.action_goal.goal.order > 0;
        }

        protected override void GoalHandler()
        {
            Console.WriteLine("Generating Fibonacci sequence of order 20 with seeds 0, 1");

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i < action.action_goal.goal.order; i++)
            {
                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();
                Console.WriteLine("Feedback @ " + DateTime.Now);
                PrintIntArray(action.action_feedback.feedback.sequence);
                Console.WriteLine("---");

                Thread.Sleep((int)(timeStep * 1000));
            }

            UpdateAndPublishStatus(ActionStatus.SUCCEEDED);
            action.action_result.result.sequence = sequence.ToArray();
            PublishResult();
            Console.WriteLine("Result @ " + DateTime.Now);
            PrintIntArray(action.action_result.result.sequence);
            Console.WriteLine("---");

            UpdateAndPublishStatus(ActionStatus.PENDING);
            Console.WriteLine("\nPress any key to stop server...\n");
        }

        private void PrintIntArray(int[] array)
        {
            Console.WriteLine("[{0}]", string.Join(", ", array));
        }
    }

    public class FibonacciActionServerConsoleExample
    {
        public static void Main(string[] args) {
            FibonacciActionConsoleServer server = new FibonacciActionConsoleServer(new FibonacciAction(), "fibonacci", Protocol.WebSocketSharp, "ws://192.168.137.195:9090");
            Console.WriteLine("Press any key to stop server...\n");
            Console.ReadKey(true);
            server.Stop();
        }
    }
}
