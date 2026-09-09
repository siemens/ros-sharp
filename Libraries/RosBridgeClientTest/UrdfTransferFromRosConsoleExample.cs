/*
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

* Reordered status events: robot description before robot name.
* Added UrdfTransferFromRos' new parameters: log, cancellationToken, maxRetries, retryDelayMs, serviceTimeoutMs
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

using System;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.UrdfTransfer;

// commands on ROS system:
// launch before starting:
// roslaunch file_server publish_description_turtlebot2.launch
// see wiki for more

namespace RosSharp.RosBridgeClientTest
{
    public class UrdfTransferFromRosConsoleExample
    {
        private static CancellationTokenSource cancellationTokenSource = new();
        public static void Main(string[] args)
        {
            string uri = "ws://localhost:9090";

            for (int i = 1; i < 3; i++)
            {
                RosBridgeClient.Protocols.WebSocketNetProtocol webSocketNetProtocol = new RosBridgeClient.Protocols.WebSocketNetProtocol(uri);
                RosSocket rosSocket = new RosSocket(webSocketNetProtocol);

#if !ROS2       // <param_name>
                string urdfParameter = "/robot_description";
                string robotNameParameter = "/robot/name";
#else           // <node_name>:<param_name>
                string urdfParameter = "robot_state_publisher:robot_description";
                string robotNameParameter = "file_server:robot_name";
#endif
                // Publication:
                UrdfTransferFromRos urdfTransferFromRos = new UrdfTransferFromRos(
                    rosSocket: rosSocket,
                    localUrdfDirectory: System.IO.Directory.GetCurrentDirectory(),
                    urdfParameter: urdfParameter,
                    robotNameParameter: robotNameParameter,
                    log: new Log(x => Console.WriteLine(x)),
                    cancellationToken: cancellationTokenSource.Token,
                    maxRetries: 3,
                    retryDelayMs: 200,
                    serviceTimeoutMs: 8000
                );

                urdfTransferFromRos.Transfer();

                urdfTransferFromRos.Status["robotDescriptionReceived"].WaitOne();
                Console.WriteLine("Robot Description received... ");

                urdfTransferFromRos.Status["robotNameReceived"].WaitOne();
                Console.WriteLine("Robot Name Received: " + urdfTransferFromRos.RobotName);

                urdfTransferFromRos.Status["resourceFilesReceived"].WaitOne();
                Console.WriteLine("Resource Files received " + urdfTransferFromRos.FilesBeingProcessed.Count);

                rosSocket.Close();
            }

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);
        }

    }
}