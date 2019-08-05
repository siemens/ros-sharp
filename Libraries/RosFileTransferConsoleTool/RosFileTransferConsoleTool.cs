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
using System.Collections.Generic;

using RosSharp.RosBridgeClient.MessageTypes.FileServer;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient.FileTransfer
{
    public class RosFileTransferConsoleTool
    {
        private static readonly string usage = 
            "Usage:\n" +
            "RosFileTransfer.exe [-h | --help] [-v | --verbose] [-r | --recursive] [-p | --package <package-name>] <source-path> <destination-path> [-ext | --extensions <array-of-extensions>] [--protocol <Sharp | NET>] [--serializer <JSON | BSON>] [--timeout <seconds-timeout>] [--timestep <seconds-timestep>]\n" +
            "    help\t\t\tPrints this message. Only valid if it is the first flag\n" +
            "    verbose\t\t\tOutputs extra information\n" +
            "    recursive\t\t\tRecursively transfers all files in given directory\n" +
            "    package\t\t\tTransfers all files of a given package\n" +
            "    extensions\t\t\tSpecifies a list of extensions to transfer\n" +
            "              \t\t\tPlease specify the array of extensions wrapped in {} and seperated by , only.\n" +
            "              \t\t\tFor example: [.msg,.srv,.action]\n" + 
            "    protocol\t\t\tSpecifies the WebSocket Protocol used. Defaults to Sharp\n" +
            "    serializer\t\t\tSpecifies the serializer used. Defaults to JSON\n" + 
            "Note:\n" +
            "- For single file transfer, we will politely ignore all extensions. Please provide extension in source path\n" +
            "- For source/destination path, start with `ROS://` to represent a path on the machine where ROS is running.\n" +
            "  - Then follow with IP and port of hosting machine for single file and directory files,\n" +
            "  - where '~' denotes the home directory of the user who started the server." +
            "  e.g. ROS://192.168.0.1:9090:~/catkin_ws \n" + 
            "  - Or follow with package anme for package files\n" + 
            "  e.g. ROS://192.168.0.1:9090:std_msgs";

        private static readonly HashSet<string> validOptions = new HashSet<string>(){
            "-h", "--help",
            "-v", "--verbose",
            "-r", "--recursive",
            "-p", "--package",
            "-ext", "--extensions",
            "--protocol",
            "--serializer",
            "--timeout",
            "--timestep"
        };

        public static void Main(string[] args)
        {
            // Arguments
            bool verbose = false;

            bool recursive = false;
            bool package = false;

            string sourcePath = "";
            string destinationPath = "";
            string[] extensions = new string[0];

            Protocol protocol = Protocol.WebSocketSharp;
            RosSocket.SerializerEnum serializer = RosSocket.SerializerEnum.JSON;

            float secondsTimestep = 0.1f;
            float secondsTimeout = 3f;

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

                if (arg.Equals("--protocol"))
                {
                    if (i == args.Length - 1)
                    {
                        Console.WriteLine("No protocol specified. Using Sharp");
                        continue;
                    }
                    else
                    {
                        switch (args[i + 1])
                        {
                            case "NET":
                                protocol = Protocol.WebSocketNET;
                                break;
                            case "Sharp":
                                protocol = Protocol.WebSocketSharp;
                                break;
                            default:
                                Console.WriteLine("Invalid protocol " + args[i + 1] + ". Ignored");
                                break;
                        }
                        i++;
                        continue;
                    }
                }

                if (arg.Equals("--serializer"))
                {
                    if (i == args.Length - 1)
                    {
                        Console.WriteLine("No serializer specified. Using JSON");
                        continue;
                    }
                    else
                    {
                        switch (args[i + 1])
                        {
                            case "JSON":
                                serializer = RosSocket.SerializerEnum.JSON;
                                break;
                            case "BSON":
                                serializer = RosSocket.SerializerEnum.BSON;
                                break;
                            default:
                                Console.WriteLine("Invalid serializer " + args[i + 1] + ". Ignored");
                                break;
                        }
                        i++;
                        continue;
                    }
                }

                if (arg.Equals("--timeout"))
                {
                    if (i == args.Length - 1)
                    {
                        Console.WriteLine("No timeout specified. Using 3 seconds");
                        continue;
                    }
                    else
                    {
                        try
                        {
                            secondsTimeout = float.Parse(args[i + 1]);
                        }
                        catch (FormatException)
                        {
                            Console.Error.WriteLine(args[i+1] + " is not a valid float value for timeout. Ignored");
                        }
                        i++;
                        continue;
                    }
                }

                if (arg.Equals("--timestep"))
                {
                    if (i == args.Length - 1)
                    {
                        Console.WriteLine("No timestep specified. Using 0.1 seconds");
                        continue;
                    }
                    else
                    {
                        try
                        {
                            secondsTimestep = float.Parse(args[i + 1]);
                        }
                        catch (FormatException)
                        {
                            Console.Error.WriteLine(args[i + 1] + " is not a valid float value for timestep. Ignored");
                        }
                        i++;
                        continue;
                    }
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

            // Do work
            // Check source/destination start with ROS:
            if (sourcePath.StartsWith("ROS://"))
            {
                FileTransferAction action = new FileTransferAction();
                FileTransferGoal goal = action.action_goal.goal;
                string[] resourceIdentifier = sourcePath.Substring(6).Split(':');
                string serverURL = "ws://" + resourceIdentifier[0] + ":" + resourceIdentifier[1];
                string identifier = resourceIdentifier[2];
                // Single File
                if (!package && !recursive) {
                    goal.type = 0;
                }
                // Package Files
                if (package) {
                    goal.type = 1;
                }
                // Directory files
                if (recursive) {
                    goal.type = 2;
                }
                goal.identifier = identifier;
                goal.extensions = extensions;

                // Create client
                Console.WriteLine(serverURL);
                FileTransferFromRosConsoleClient client = new FileTransferFromRosConsoleClient(action, destinationPath, serverURL, protocol, serializer, secondsTimeout, secondsTimestep, verbose);
                client.Execute();
            }
        }
    }
}
