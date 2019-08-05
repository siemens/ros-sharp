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

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.FileServer;

namespace RosSharp.RosBridgeClient.FileTransfer
{
    public class FileTransferFromRosConsoleClient : FileTransferFromRosClient
    {
        public FileTransferFromRosConsoleClient(FileTransferAction action, string outPath, string serverURL, Protocol protocol = Protocol.WebSocketSharp, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float secondsTimeout = 3f, float secondsTimestep = 0.1f, bool verbose = false) : base(action, outPath, serverURL, protocol, serializer, secondsTimeout, secondsTimestep, verbose) { }

        protected override void Log(string log)
        {
            Console.WriteLine("File Transfer from ROS Client @ " + DateTime.Now + " : [LOG] " + log);
        }

        protected override void LogWarning(string log)
        {
            Console.WriteLine("File Transfer from ROS Client @ " + DateTime.Now + " : [WARNING] " + log);
        }

        protected override void LogError(string log)
        {
            Console.Error.WriteLine("File Transfer from ROS Client @ " + DateTime.Now + " : [ERROR] " + log);
        }
    }
}
