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
    public class FibonacciActionServer : ActionServer<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        public FibonacciActionServer(FibonacciAction action, string actionName, Protocol protocol, string serverURL) : base(action, actionName, protocol, serverURL) { }

        protected override void GoalHandler(FibonacciActionGoal actionGoal)
        {
            int order = actionGoal.goal.order;

            if (actionGoal.goal.order <= 0) {
                UpdateAndPublishStatus(ActionStatus.REJECTED);
                Console.WriteLine("Sorry, I cannot give you a subsequence of length less than 1");
            }

            base.GoalHandler(actionGoal);
            
            UpdateAndPublishStatus(ActionStatus.ACTIVE);

            List<int> sequence = new List<int> { 0, 1 };

            action.action_feedback.feedback.sequence = sequence.ToArray();
            PublishFeedback();

            for (int i = 1; i <= actionGoal.goal.order; i++)
            {
                sequence.Add(sequence[i] + sequence[i - 1]);
                action.action_feedback.feedback.sequence = sequence.ToArray();
                PublishFeedback();
                Thread.Sleep(1000);
            }

            UpdateAndPublishStatus(ActionStatus.SUCCEEDED);
            action.action_result.result.sequence = sequence.ToArray();
            PublishResult();
        }
    }

    public class FibonacciActionServerConsoleExample
    {
        public static void Main(string[] args) {
            FibonacciActionServer server = new FibonacciActionServer(new FibonacciAction(), "fibonacci", Protocol.WebSocketSharp, "ws://192.168.137.195:9090");
            Console.WriteLine("Press any key to stop server...");
            Console.ReadKey(true);
            server.Stop();
        }
    }
}
