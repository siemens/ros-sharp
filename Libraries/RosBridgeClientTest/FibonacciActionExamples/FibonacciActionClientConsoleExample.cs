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

using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials;

namespace RosSharp.RosBridgeClientTest
{
    public class FibonacciActionConsoleClient : ActionClient<FibonacciAction, FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        private ManualResetEvent isResultReceived = new ManualResetEvent(false);

        public FibonacciActionConsoleClient(FibonacciAction action, string actionName, string serverURL) : base(action, actionName, serverURL) { }

        public void Execute() {
            Start();

            Log("Waiting for action server...");
            WaitForActionServer();

            SendGoal();

            Log("Waiting for result...");
            WaitForResult();

            Stop();
        }

        protected override string GoalID()
        {
            return "fibonacci-sharp-console-" + Guid.NewGuid();
        }

        protected void WaitForActionServer()
        {
            while((DateTime.Now - lastStatusUpdateTime).TotalMilliseconds > millisecondsTimeout) {
                Thread.Sleep(millisecondsTimestep);
            }
        }

        protected void WaitForResult()
        {
            while (!isResultReceived.WaitOne(0))
            {
                Thread.Sleep(millisecondsTimestep);
            }
        }

        protected override void OnFeedbackReceived()
        {
            Log(action.action_feedback.ToString());
        }

        protected override void OnResultReceived()
        {
            Log(action.action_result.ToString());
            isResultReceived.Set();
        }

        protected override void Log(string log)
        {
            Console.WriteLine("Fibonacci Action Client @ " + DateTime.Now + " : [LOG] " + log);
        }

        protected override void LogWarning(string log)
        {
            Console.WriteLine("Fibonacci Action Client @ " + DateTime.Now + " : [WARNING] " + log);
        }

        protected override void LogError(string log)
        {
            Console.Error.WriteLine("Fibonacci Action Client @ " + DateTime.Now + " : [ERROR] " + log);
        }
    }

    public class FibonacciActionClientConsoleExample
    {
        public static void Main(string[] args) {
            FibonacciAction action = new FibonacciAction();
            action.action_goal.goal.order = 20;
            FibonacciActionConsoleClient client = new FibonacciActionConsoleClient(action, "fibonacci", "ws://192.168.137.195:9090");
            client.Execute();
        }
    }
}
