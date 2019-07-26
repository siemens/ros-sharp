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
using System.Collections.Concurrent;
using System.IO;
using System.Threading;

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.FileServer;

namespace RosSharp.RosBridgeClient.FileTransfer
{
    public class FileTransferFromRosConsoleClient : ActionClient<FileTransferFromRosAction, FileTransferFromRosActionGoal, FileTransferFromRosActionResult, FileTransferFromRosActionFeedback, FileTransferFromRosGoal, FileTransferFromRosResult, FileTransferFromRosFeedback>
    {
        private readonly string outPath;

        private ConcurrentQueue<FileTransferFromRosFeedback> files;

        private readonly int serverWaitTimeout;
        private ManualResetEvent isResultReceived = new ManualResetEvent(false);

        public FileTransferFromRosConsoleClient(FileTransferFromRosAction action, string outPath, string serverURL, Protocol protocol = Protocol.WebSocketSharp, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float timeStep = 0.2f, int serverWaitTimeout = 3) : base(action, "file_transfer_from_ros", serverURL, protocol, serializer, timeStep)
        {
            this.outPath = outPath;
            this.serverWaitTimeout = serverWaitTimeout;

            files = new ConcurrentQueue<FileTransferFromRosFeedback>();
        }

        public void Execute()
        {
            Start();

            Console.WriteLine("Wait for server...");
            WaitForActionServer();

            SendGoal();

            WriteFiles();

            Console.WriteLine("No more files to receive. Flushing files queue");
            FlushFilesQueue();

            Stop();
        }

        protected override void WaitForActionServer()
        {
            while ((DateTime.Now - lastStatusUpdateTime).TotalSeconds > serverWaitTimeout)
            {
                Thread.Sleep((int)(timeStep * 1000));
            }
        }

        protected override void WaitForResult()
        {
            // Left empty since write files will spin
        }

        protected override void FeedbackHandler()
        {
            files.Enqueue(action.action_feedback.feedback);
        }

        protected override void ResultHandler()
        {
            isResultReceived.Set();
        }

        private string GetCompleteOutPath()
        {
            string[] rosPathStructure = action.action_feedback.feedback.path.Split('/');
            string extendedOutPath = outPath;

            // Get output path
            switch (action.action_goal.goal.type)
            {
                case 0:
                    // Single File
                    break;
                case 1:
                    // Package
                    int indexOfPackageName = Array.IndexOf(rosPathStructure, action.action_goal.goal.identifier);
                    for (int i = indexOfPackageName; i < rosPathStructure.Length - 1; i++)
                    {
                        extendedOutPath = Path.Combine(extendedOutPath, rosPathStructure[i]);
                    }
                    if (!Directory.Exists(extendedOutPath))
                    {
                        Directory.CreateDirectory(extendedOutPath);
                    }
                    extendedOutPath = Path.Combine(extendedOutPath, rosPathStructure[rosPathStructure.Length - 1]);
                    break;
                case 2:
                    // Recursive
                    break;
            }

            return extendedOutPath;
        }

        private void WriteFiles()
        {
            while (!isResultReceived.WaitOne(0) || !files.IsEmpty)
            {
                if (files.TryDequeue(out FileTransferFromRosFeedback file))
                {
                    string completeOutPath = GetCompleteOutPath();
                    File.WriteAllBytes(completeOutPath, file.content);
                    Console.WriteLine("(" + file.number + "/" + file.count + ") " + completeOutPath);
                }
            }
        }

        private void FlushFilesQueue()
        {
            while (!files.IsEmpty)
            {
                if (files.TryDequeue(out FileTransferFromRosFeedback file))
                {
                    string completeOutPath = GetCompleteOutPath();
                    File.WriteAllBytes(completeOutPath, file.content);
                    Console.WriteLine("(" + file.number + "/" + file.count + ") " + completeOutPath);
                }
            }
        }
    }

    public class FileTransferFromRos
    {
        public static void Main(string[] args)
        {
            // Package Transfer as example
            FileTransferFromRosAction action = new FileTransferFromRosAction();
            action.action_goal.goal.type = 1;
            action.action_goal.goal.identifier = "std_msgs";
            action.action_goal.goal.types = new string[] {".msg", ".srv", ".action"};

            FileTransferFromRosConsoleClient client = new FileTransferFromRosConsoleClient(action, "C:\\", "ws://192.168.137.195:9090");
            client.Execute();
        }
    }
}
