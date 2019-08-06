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

using System.Collections.Concurrent;
using System.Threading;

using UnityEngine;

using RosSharp.RosBridgeClient.FileTransfer;
using RosSharp.RosBridgeClient.MessageTypes.FileServer;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient.RosFileTransfer
{
    public class FileTransferFromRosUnityClient : FileTransferFromRosClient
    {
        public FileTransferFromRosUnityClient(FileTransferAction action, string outPath, string serverURL, Protocol protocol = Protocol.WebSocketSharp, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float secondsTimeout = 3f, float secondsTimestep = 0.1f, bool verbose = false) : base(action, outPath, serverURL, protocol, serializer, secondsTimeout, secondsTimestep, verbose) { }

        public void UnityWaitForServer(ManualResetEvent isWaitingForServer, ManualResetEvent shouldWaitForServer)
        {
            isWaitingForServer.Set();
            CancellableWaitForActionServer(shouldWaitForServer);
            isWaitingForServer.Reset();
        }

        public void UnitySetGoal(FileTransferGoal goal)
        {
            action.action_goal.goal = goal;
        }

        public void UnitySendGoal(ManualResetEvent isSendingGoal)
        {
            isSendingGoal.Set();
            SendGoal();
            isSendingGoal.Reset();
        }

        public void UnityCancelGoal()
        {
            CancelGoal();
        }

        public ConcurrentQueue<FileTransferFeedback> UnityGetFiles()
        {
            return GetFiles();
        }

        public bool UnityIsResultReceived()
        {
            return IsResultReceived();
        }

        public string UnityGetCompleteOutPath(FileTransferFeedback file)
        {
            return GetCompleteOutPath(file);
        }

        protected override void Log(string log)
        {
            Debug.Log("File Transfer from ROS Client: " + log);
        }

        protected override void LogWarning(string log)
        {
            Debug.LogWarning("File Transfer from ROS Client: " + log);
        }

        protected override void LogError(string log)
        {
            Debug.LogError("File Transfer from ROS Client: " + log);
        }
    }
}

