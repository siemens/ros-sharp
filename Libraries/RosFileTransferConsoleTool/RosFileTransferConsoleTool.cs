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
using System.Collections.Generic;
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
                Thread.Sleep(millisecondsTimestep);
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

    public class RosFileTransferConsoleTool
    {
        private static readonly string usage = 
            "Usage:\n" +
            "RosFileTransfer.exe [-h | --help] [-v | --verbose] [-r | --recursive] [-p | --package <package-name>] <source-path> <destination-path> [-ext | --extensions <array-of-extensions>]\n" +
            "    help\t\t\tPrints this message. Only valid if it is the first flag\n" +
            "    verbose\t\t\tOutputs extra information\n" +
            "    recursive\t\t\tRecursively transfers all files in given directory\n" +
            "    package\t\t\tTransfers all files of a given package\n" +
            "    extensions\t\t\tSpecifies a list of extensions to transfer\n" +
            "              \t\t\tPlease specify the array of extensions wrapped in {} and seperated by , only.\n" +
            "              \t\t\tFor example: [msg,srv,action]\n" + 
            "Note:\n" +
            "- For single file transfer, we will politely ignore all extensions. Please provide extension in source path\n" +
            "- For source/destination path, start with `ROS:` to represent a path on the machine where ROS is running.\n" +
            "- For ROS path, please use UNIX path format";

        private static readonly HashSet<string> validOptions = new HashSet<string>(){
            "-h", "--help",
            "-v", "--verbose",
            "-r", "--recursive",
            "-p", "--package",
            "-ext", "--extensions"
        };

        public static void Main(string[] args)
        {
            // Arguments
            bool verbose = false;

            bool recursive = false;
            bool package = false;

            string sourcePath = "";
            string destinationPath = "";
            string[] extensions;

            // Parse Arguments
            if (args.Length == 0)
            {
                Console.WriteLine("No argument: Display usage");
                Console.WriteLine(usage);
                return;
            }

            if (args[0].Equals("--help") || args[0].Equals("-h"))
            {
                Console.WriteLine("Help: Displays usage");
                Console.WriteLine(usage);
                return;
            }

            for (int i = 0; i < args.Length; i++)
            {
                string arg = args[i];

                if (arg.Equals("-v") || arg.Equals("--verbose"))
                {
                    Console.WriteLine("Verbose flag detected. Warning: A huge wave of text incoming!");
                    verbose = true;
                    continue;
                }

                if (arg.Equals("-r") || arg.Equals("--recursive"))
                {
                    if (package)
                    {
                        Console.Error.WriteLine("Option '--recursive' conflict with '--package'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Make up your mind!");
                        }
                        return;
                    }
                    if (recursive)
                    {
                        Console.WriteLine("Duplicate flag '--recursive'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("I got it already!");
                        }
                    }
                    recursive = true;
                    continue;
                }

                if (arg.Equals("-p") || arg.Equals("--package"))
                {
                    if (recursive)
                    {
                        Console.Error.WriteLine("Option '--package' conflict with '--recursive'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Make up your mind!");
                        }
                        return;
                    }
                    if (package)
                    {
                        Console.WriteLine("Duplicate flag '--package'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("I got it already!");
                        }
                    }
                    package = true;
                    continue;
                }

                if (arg.Equals("-ext") || arg.Equals("--extensions"))
                {
                    if (!package && !recursive)
                    {
                        Console.WriteLine("Single file mode, ignoring extensions parameter.");
                        continue;
                    }
                    if (i == args.Length - 1)
                    {
                        Console.WriteLine("No extensions array, getting all files");
                        continue;
                    }

                    string extStr = args[i + 1];
                    if (extStr.StartsWith("[") && extStr.EndsWith("]"))
                    {
                        extStr = extStr.Substring(1, extStr.Length - 2);
                        extensions = extStr.Split(',');
                        i++;
                        if (verbose)
                        {
                            Console.WriteLine("Extensions: " + String.Join(",", extensions));
                        }
                    }
                    else
                    {
                        Console.WriteLine("Unexpected extension array format. Ignored. Getting all files");
                    }
                    continue;
                }

                if (sourcePath.Equals(""))
                {
                    sourcePath = arg;
                    continue;
                }

                if (destinationPath.Equals(""))
                {
                    destinationPath = arg;
                    continue;
                }

                Console.WriteLine("Ignored invalid argument: " + arg);
                if (verbose)
                {
                    Console.WriteLine("Mumble mumble");
                }
            }
        }
    }
}
