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

using System;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;

namespace RosSharp.RosBridgeClientTest
{
    class FibonacciActionServerConsoleExample
    {
        static readonly string uri = "ws://10.42.0.1:9090";
        static readonly string actionName = "fibonacci";

        private static RosSocket rosSocket;
        private static ManualResetEvent isWaitingForGoal = new ManualResetEvent(false);
        private static FibonacciActionServer fibonacciActionServer;

        public static void Main(string[] args)
        {
            //RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketSharpProtocol(uri));
            rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

            // Initialize server
            fibonacciActionServer = new FibonacciActionServer(actionName, rosSocket, new MessageLogger(new MessageLogger.LogDelegate(x => Console.WriteLine(x))));
            fibonacciActionServer.Initialize();

            // Run server and wait for goal
            isWaitingForGoal.Set();
            Console.WriteLine("Waiting for goal");
            Thread waitForGoal = new Thread(WaitForGoal);
            waitForGoal.Start();

            Console.WriteLine("\nPress any key to stop server...");
            Console.ReadKey(true);
            isWaitingForGoal.Reset();
            waitForGoal.Join();
            fibonacciActionServer.Terminate();

            // End of console example
            Console.WriteLine("\nPress any key to close...");
            Console.ReadKey(true);
            rosSocket.Close();
        }

        private static void WaitForGoal()
        {
            while (isWaitingForGoal.WaitOne(0))
            {
                fibonacciActionServer.PublishStatus();
                Thread.Sleep(50);
            }
        }
    }
}