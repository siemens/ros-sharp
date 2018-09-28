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
            string uri = "ws://192.168.56.102:9090";
            string robotDescription = Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "TestUrdf", "R2D2WithTexture.urdf");

            WebSocketNetProtocol webSocketNetProtocol = new WebSocketNetProtocol(uri);
            RosSocket rosSocket = new RosSocket(webSocketNetProtocol);

            // Publication:
            UrdfTransferToRos transferor = new UrdfTransferToRos(rosSocket, "Robot", robotDescription, "test_package");
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