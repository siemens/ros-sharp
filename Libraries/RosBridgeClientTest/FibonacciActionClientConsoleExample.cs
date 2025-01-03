/*
© Siemens AG, 2019
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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

#if !ROS2
using System;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClientTest
{
    class FibonacciActionClientConsoleExample
    {
        static readonly string uri = "ws://10.42.0.1:9090";
        static readonly string actionName = "fibonacci";

        private static RosSocket rosSocket;
        private static FibonacciActionClient fibonacciActionClient;

        public static void Main(string[] args)
        {
            //RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketSharpProtocol(uri));
            rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

            // Initialize Client
            fibonacciActionClient = new FibonacciActionClient(actionName, rosSocket);
            fibonacciActionClient.Initialize();

            // Send goal
            Console.WriteLine("\nPress any key to send goal with fibonacci order 5...");
            Console.ReadKey(true);
            fibonacciActionClient.fibonacciOrder = 5;
            fibonacciActionClient.SendGoal();

            // Get feedback, status and result
            do
            {
                Console.WriteLine(fibonacciActionClient.GetFeedbackString());
                Console.WriteLine(fibonacciActionClient.GetStatusString());
                Thread.Sleep(100);
            } while (fibonacciActionClient.goalStatus.status != GoalStatus.SUCCEEDED);

            Thread.Sleep(500);
            Console.WriteLine(fibonacciActionClient.GetResultString());
            Console.WriteLine(fibonacciActionClient.GetStatusString());

            // Cancel goal
            Console.WriteLine("\nPress any key to send goal with fibonacci order 50...");
            Console.ReadKey(true);
            fibonacciActionClient.fibonacciOrder = 50;
            fibonacciActionClient.SendGoal();

            Console.WriteLine("\nPress any key to cancel the goal...");
            Console.ReadKey(true);
            fibonacciActionClient.CancelGoal();
            Thread.Sleep(1000);
            Console.WriteLine(fibonacciActionClient.GetResultString());
            Console.WriteLine(fibonacciActionClient.GetFeedbackString());
            Console.WriteLine(fibonacciActionClient.GetStatusString());

            // Terminate client
            fibonacciActionClient.Terminate();

            // End of console example
            Console.WriteLine("\nPress any key to close...");
            Console.ReadKey(true);
            rosSocket.Close();
        }

    }
}
#endif