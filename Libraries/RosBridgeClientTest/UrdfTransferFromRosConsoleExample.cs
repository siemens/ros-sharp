﻿/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.UrdfTransfer;

// commands on ROS system:
// launch before starting:
// roslaunch file_server publish_description_turtlebot2.launch

namespace RosSharp.RosBridgeClientTest
{
    public class UrdfTransferFromRosConsoleExample
    {
        public static void Main(string[] args)
        {
            string uri = "ws://192.168.56.102:9090";

            for (int i = 1; i < 3; i++)
            {
                var webSocketNetProtocol = new RosBridgeClient.Protocols.WebSocketNetProtocol(uri);
                var rosSocket = new RosSocket(webSocketNetProtocol);

                // Publication:
                UrdfTransferFromRos transferor = new UrdfTransferFromRos(rosSocket, System.IO.Directory.GetCurrentDirectory());
                transferor.Transfer();

                transferor.Status["robotNameReceived"].WaitOne();
                Console.WriteLine("Robot Name Received: " + transferor.RobotName);

                transferor.Status["robotDescriptionReceived"].WaitOne();
                Console.WriteLine("Robot Description received... ");

                transferor.Status["resourceFilesReceived"].WaitOne();
                Console.WriteLine("Resource Files received " + transferor.FilesBeingProcessed.Count);

                rosSocket.Close();
            }

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);
        }

    }
}