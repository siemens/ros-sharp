/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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
using System.IO;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.UrdfTransfer;

namespace RosSharp.RosBridgeClientTest
{
    public class UrdfTransferToRosConsoleExample
    {
        // on ROS system:
        // launch before starting:
        // roslaunch file_server ros_sharp_communication.launch

        // launch after starting:
        // roslaunch file_server visualize_robot.launch

        public static void Main(string[] args)
        {
            string uri = "ws://localhost:9090";
            string urdfFilePath = Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "<package_name>", "<urdf_name>.urdf");

#if ROS2
            string robotNameParameter = "r2d2:urdf_tutorial"; // <node_name>:<param_name>
#else
            string robotNameParameter = "robot_name"; // <param_name>
#endif

            WebSocketNetProtocol webSocketNetProtocol = new WebSocketNetProtocol(uri);
            RosSocket rosSocket = new RosSocket(webSocketNetProtocol);

            // Publication:
            UrdfTransferToRos transferor = new UrdfTransferToRos(rosSocket, "Robot", robotNameParameter, urdfFilePath, "<package_name_to_be_exported>"); //urdf_export_test
            transferor.Transfer();

            transferor.Status["robotNamePublished"].WaitOne();
            Console.WriteLine("Robot Name Published: " + transferor.RobotName);

            transferor.Status["robotDescriptionPublished"].WaitOne();
            Console.WriteLine("Robot Description received... ");

            transferor.Status["resourceFilesSent"].WaitOne();
            Console.WriteLine("Resource Files sent: " + transferor.FilesBeingProcessed.Count);

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);

            rosSocket.Close();
        }

    }
}