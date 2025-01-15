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

- Added ROS2 action support
    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

using System;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;

namespace RosSharp.RosBridgeClientTest
{
    class FibonacciActionServerConsoleExample
    {
        static readonly string uri = "ws://localhost:9090";
        static readonly string actionName = "/fibonacci";

        private static RosSocket rosSocket;
        private static FibonacciActionServer fibonacciActionServer;

#if !ROS2
        private static ManualResetEvent isWaitingForGoal = new ManualResetEvent(false);
#endif

        public static void Main(string[] args)
        {
            rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

            // Initialize server
            fibonacciActionServer = new FibonacciActionServer(actionName, rosSocket, new Log(x => Console.WriteLine(x)));
            fibonacciActionServer.Initialize();

#if !ROS2

            // Run server and wait for goal
            isWaitingForGoal.Set();
            Console.WriteLine("Waiting for goal");
            Thread waitForGoal = new Thread(WaitForGoal);
            waitForGoal.Start();

            Console.WriteLine("\nPress any key to stop server.");
            Console.ReadKey(true);
            isWaitingForGoal.Reset();
            waitForGoal.Join();
            fibonacciActionServer.Terminate();
#else
            Console.WriteLine("Waiting for goal. Press any key to stop server.");
            Console.ReadKey(true);

            fibonacciActionServer.Terminate();
            rosSocket.Close();
#endif
            // End of console example
            Console.WriteLine("\nPress any key to close.");
            Console.ReadKey(true);
            rosSocket.Close();

        }
#if !ROS2
        private static void WaitForGoal()
        {
            while (isWaitingForGoal.WaitOne(0))
            {
                fibonacciActionServer.PublishStatus();
                Thread.Sleep(50);
            }
        }
#endif
    }
}
