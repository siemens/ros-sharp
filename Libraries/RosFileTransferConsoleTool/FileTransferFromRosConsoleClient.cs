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
    public class FileTransferFromRosConsoleClient : ActionClient<FileTransferAction, FileTransferActionGoal, FileTransferActionResult, FileTransferActionFeedback, FileTransferGoal, FileTransferResult, FileTransferFeedback>
    {
        private readonly string outPath;

        private ConcurrentQueue<FileTransferFeedback> files;

        private readonly int serverWaitTimeout;
        private ManualResetEvent isResultReceived = new ManualResetEvent(false);

        public FileTransferFromRosConsoleClient(FileTransferAction action, string outPath, string serverURL, Protocol protocol = Protocol.WebSocketSharp, RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON, float timeStep = 0.1f, int serverWaitTimeout = 3) : base(action, "file_transfer_from_ros", serverURL, protocol, serializer, timeStep)
        {
            this.outPath = outPath;
            this.serverWaitTimeout = serverWaitTimeout;

            files = new ConcurrentQueue<FileTransferFeedback>();
        }

        public void Execute()
        {
            Start();

            Console.WriteLine("Wait for server...");
            WaitForActionServer();

            SendGoal();

            Thread writeFiles = new Thread(WriteFiles);
            writeFiles.Start();

            writeFiles.Join();
            FlushFilesQueue();

            Stop();
        }

        protected override string GoalID()
        {
            return GenRandomGoalID("file-transfer-from-ros-console-");
        }

        protected override void WaitForActionServer()
        {
            while ((DateTime.Now - lastStatusUpdateTime).TotalSeconds > serverWaitTimeout)
            {
                Thread.Sleep(millisecondsTimestep);
            }
        }

        protected override void WaitForResult()
        {
            // Left empty since write files will spin
        }

        protected override void FeedbackHandler()
        {
            new Thread(() => files.Enqueue(action.action_feedback.feedback)).Start();
        }

        protected override void StatusHandler()
        {
            if (goalStatus != null) {
                if (goalStatus.status == (byte)ActionStatus.ABORTED)
                {
                    Console.Error.WriteLine(goalStatus.text);
                }
            }
        }

        protected override void ResultHandler()
        {
            isResultReceived.Set();
            if (action.action_result.status.status == (byte)ActionStatus.ABORTED) {
                Console.Error.WriteLine(action.action_result.status.text);
            }
        }

        private string GetCompleteOutPath(FileTransferFeedback file)
        {
            string[] rosPathStructure = file.path.Split('/');
            string[] goalPathStructure = action.action_goal.goal.identifier.Split('/');
            string extendedOutPath = outPath;

            // Get output path
            switch (action.action_goal.goal.type)
            {
                case 0:
                    // Single File
                    // Don't need to do anything here
                    break;
                case 1:
                    // Package
                    int indexOfPackageName = Array.IndexOf(rosPathStructure, action.action_goal.goal.identifier);
                    for (int i = indexOfPackageName; i < rosPathStructure.Length - 1; i++)
                    {
                        extendedOutPath = Path.Combine(extendedOutPath, rosPathStructure[i]);
                    }
                    break;
                case 2:
                    // Recursive
                    string dirName = goalPathStructure[goalPathStructure.Length - 1];
                    int indexOfDirName = Array.IndexOf(rosPathStructure, dirName);
                    for (int i = indexOfDirName; i < rosPathStructure.Length - 1; i++) {
                        extendedOutPath = Path.Combine(extendedOutPath, rosPathStructure[i]);
                    }
                    break;
            }
            // Create directory if it doesn't exist yet
            if (!Directory.Exists(extendedOutPath))
            {
                Directory.CreateDirectory(extendedOutPath);
            }
            // Append file name
            extendedOutPath = Path.Combine(extendedOutPath, rosPathStructure[rosPathStructure.Length - 1]);

            return extendedOutPath;
        }

        private void WriteFiles()
        {
            while (!isResultReceived.WaitOne(0) || !files.IsEmpty)
            {
                if (files.TryDequeue(out FileTransferFeedback file))
                {
                    string completeOutPath = GetCompleteOutPath(file);
                    File.WriteAllBytes(completeOutPath, file.content);
                    Console.WriteLine("(" + file.number + "/" + file.count + ") " + completeOutPath);
                }
                Thread.Sleep(millisecondsTimestep);
            }
        }

        private void FlushFilesQueue()
        {
            Console.WriteLine("Flushing " + files.Count + " files");
            while (!files.IsEmpty)
            {
                if (files.TryDequeue(out FileTransferFeedback file))
                {
                    string completeOutPath = GetCompleteOutPath(file);
                    File.WriteAllBytes(completeOutPath, file.content);
                    Console.WriteLine("(" + file.number + "/" + file.count + ") " + completeOutPath);
                }
                Thread.Sleep(millisecondsTimestep);
            }
            Console.WriteLine("Flushed.");
        }
    }
}
