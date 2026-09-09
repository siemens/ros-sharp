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

- Added ROS1 version switch toggle. (--ros1)
    © Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com)

- Improved command-line help, status, warning, and error messages.
    © Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com)
*/

using System;
using System.IO;
using System.Security;

using System.Collections.Generic;

using RosSharp.RosBridgeClient.MessageGeneration;

namespace RosSharp.RosBridgeClient.MessageGenerationConsoleTool
{
    public class RosMsgGen
    {
        private static readonly string usage =
            "Usage:\n" +
            "RosMsgGen.exe [-h | --help] [-v | --verbose] [-s | --service] [-a | --action] [-r | --recursive] [-p | --package] [-n | --ros-package-name <package-name>] [-o | --output <output-path>] [--ros1]\n" +
            "    help\t\t\tPrints this message. Only valid if it is the first flag\n" +
            "    verbose\t\t\tOutputs extra information\n" +
            "    service\t\t\tGenerate service messages\n" +
            "    action\t\t\tGenerate action messages\n" +
            "    recursive\t\t\tGenerate message for all ROS message files in the specified directory\n" +
            "    package\t\t\tTreat the directory as a single ROS package during message generation\n" +
            "    ros-package-name\t\tSpecify the ROS package name for the message\n" +
            "                    \t\tIf unspecified, package name will be retrieved from path, assuming ROS package structure\n" +
            "    output\t\t\tSpecify output path\n" +
            "          \t\t\tIf unspecified, output will be in current working directory, under RosSharpMessages\n" +
            "    ros1\t\t\t\tGenerate for ROS1 instead of ROS2\n\n" +
            "Note:\n" +
            "- std_msgs/Time and std_msgs/Duration will not be generated since they need to be defined with primitive variables\n" +
            "- The Message abstract class will also not be generated, but is required since all generated message classes inherit it\n" +
            "Those can be found at the ROS# GitHub repo <https://github.com/siemens/ros-sharp>\n";

        private static readonly HashSet<string> validOptions = new HashSet<string>(){
            "-h", "--help",
            "-v", "--verbose",
            "-s", "--service",
            "-a", "--action",
            "-r", "--recursive",
            "-p", "--package",
            "-n", "--ros-package-name",
            "-o", "--output",
            "--ros1"
        };

        private static readonly string defaultOutputDirectory = Path.Combine(Directory.GetCurrentDirectory(), "RosSharpMessages");

        public static void Main(string[] args)
        {
            // Arguments
            bool verbose = false;

            bool service = false;
            bool action = false;

            bool recursive = false;
            bool package = false;

            string rosPackageName = "";
            string inputPath = "";
            string outputPath = defaultOutputDirectory;

            // Parse Arguments
            if (args.Length == 0)
            {
                Console.WriteLine("No arguments provided. Displaying usage information.");
                Console.WriteLine(usage);
                return;
            }

            if (args[0].Equals("--help") || args[0].Equals("-h"))
            {
                Console.WriteLine("Displaying usage information.");
                Console.WriteLine(usage);
                return;
            }

            for (int i = 0; i < args.Length; i++)
            {
                string arg = args[i];

                if (arg.Equals("--ros1"))
                {
                    if (verbose)
                    {
                        Console.WriteLine("Detected '--ros1' flag. Generating for ROS1.");
                    }
                    if (action)
                    {
                        ActionAutoGen.isRos2 = false;
                    }
                    else if (service)
                    {
                        ServiceAutoGen.isRos2 = false;
                    }
                    else
                    {
                        MessageAutoGen.isRos2 = false;
                    }

                    continue;
                }

                if (arg.Equals("-s") || arg.Equals("--service"))
                {
                    if (action)
                    {
                        Console.Error.WriteLine("Option '--service' conflicts with '--action'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Please specify only one of these options.");
                        }

                        return;
                    }

                    service = true;
                    if (verbose)
                    {
                        Console.WriteLine("Detected '--service' flag. Expecting '*.srv' input.");
                    }

                    continue;
                }

                if (arg.Equals("-a") || arg.Equals("--action"))
                {
                    if (service)
                    {
                        Console.Error.WriteLine("Option '--action' conflicts with '--service'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Please specify only one of these options.");
                        }

                        return;
                    }

                    action = true;
                    if (verbose)
                    {
                        Console.WriteLine("Detected '--action' flag. Expecting '*.action' input.");
                    }

                    continue;
                }

                if (arg.Equals("-r") || arg.Equals("--recursive"))
                {
                    if (package)
                    {
                        Console.Error.WriteLine("Option '--recursive' conflicts with '--package'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Please specify only one of these options.");
                        }

                        return;
                    }

                    if (recursive)
                    {
                        Console.WriteLine("Duplicate '--recursive' flag detected.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Option '--recursive' has already been specified.");
                        }
                    }

                    if (!rosPackageName.Equals(""))
                    {
                        Console.WriteLine("'--recursive' option specified; provided ROS package name will be ignored.");
                    }

                    recursive = true;
                    continue;
                }

                if (arg.Equals("-p") || arg.Equals("--package"))
                {
                    if (recursive)
                    {
                        Console.Error.WriteLine("Option '--package' conflicts with '--recursive'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Please specify only one of these options.");
                        }

                        return;
                    }

                    if (package)
                    {
                        Console.WriteLine("Duplicate '--package' flag detected.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Option '--package' has already been specified.");
                        }
                    }

                    package = true;
                    continue;
                }

                if (arg.Equals("-n") || arg.Equals("--ros-package-name"))
                {
                    if (i == args.Length - 1)
                    {
                        Console.Error.WriteLine("Missing ROS package name; will infer package name from the provided path.");
                    }
                    else if (validOptions.Contains(args[i + 1]))
                    {
                        Console.Error.WriteLine("Missing ROS package name; will infer package name from the provided path.");
                    }
                    else if (recursive)
                    {
                        Console.WriteLine("'--recursive' option specified; provided ROS package name will be ignored.");
                    }
                    else
                    {
                        rosPackageName = args[i + 1];
                        i++;
                    }

                    continue;
                }

                if (arg.Equals("-o") || arg.Equals("--output"))
                {
                    if (i == args.Length - 1)
                    {
                        Console.Error.WriteLine("Missing output path; using default output path.");
                    }
                    else if (validOptions.Contains(args[i + 1]))
                    {
                        Console.Error.WriteLine("Missing output path; using default output path.");
                    }
                    else
                    {
                        outputPath = args[i + 1];
                        outputPath = Path.Combine(outputPath, "RosSharpMessages");
                        i++;
                    }

                    continue;
                }

                if (arg.Equals("-v") || arg.Equals("--verbose"))
                {
                    Console.WriteLine("Verbose mode enabled. Detailed output will be displayed.");
                    verbose = true;
                    continue;
                }

                if (inputPath.Equals(""))
                {
                    inputPath = arg;
                }
                else
                {
                    Console.Error.WriteLine("Ignored unrecognized argument: " + arg);
                    if (verbose)
                    {
                        Console.Error.WriteLine("Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage.");
                    }
                }
            }

            // Do work
            // Parse Individual Messages
            if (!package && !recursive)
            {
                if (inputPath.Equals(""))
                {
                    Console.Error.WriteLine("Please specify an input file.");
                    if (verbose)
                    {
                        Console.Error.WriteLine("Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage.");
                    }

                    return;
                }

                if (!IsValidPath(inputPath, false))
                {
                    if (IsValidPath(inputPath, true))
                    {
                        Console.Error.WriteLine("Input path is a directory. Use '--package' or '--recursive' option.");
                    }
                    else
                    {
                        Console.Error.WriteLine("Invalid input path.");
                    }

                    return;
                }

                List<string> warnings;
                if (service)
                {
                    warnings = ServiceAutoGen.GenerateSingleService(inputPath, outputPath, rosPackageName, verbose);
                }
                else if (action)
                {
                    warnings = ActionAutoGen.GenerateSingleAction(inputPath, outputPath, rosPackageName, verbose);
                }
                else
                {
                    warnings = MessageAutoGen.GenerateSingleMessage(inputPath, outputPath, rosPackageName, verbose);
                }

                PrintWarnings(warnings);
                return;
            }

            // Parse Package Messages
            if (package)
            {
                if (inputPath.Equals(""))
                {
                    Console.Error.WriteLine("Please specify an input package.");
                    if (verbose)
                    {
                        Console.Error.WriteLine("Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage.");
                    }

                    return;
                }

                if (!IsValidPath(inputPath, true))
                {
                    if (IsValidPath(inputPath, false))
                    {
                        Console.Error.WriteLine("Input path is a file. Remove the '--package' option.");
                    }
                    else
                    {
                        Console.Error.WriteLine("Invalid input path.");
                    }

                    return;
                }

                try
                {
                    Console.WriteLine("Processing package...");
                    List<string> warnings;
                    if (service)
                    {
                        warnings = ServiceAutoGen.GeneratePackageServices(inputPath, outputPath, rosPackageName, verbose);
                    }
                    else if (action)
                    {
                        warnings = ActionAutoGen.GeneratePackageActions(inputPath, outputPath, rosPackageName, verbose);
                    }
                    else
                    {
                        warnings = MessageAutoGen.GeneratePackageMessages(inputPath, outputPath, rosPackageName, verbose);
                    }

                    PrintWarnings(warnings);
                }
                catch (DirectoryNotFoundException)
                {
                    if (service)
                    {
                        Console.Error.WriteLine("Service folder not found in the specified package.");
                    }
                    else
                    {
                        Console.Error.WriteLine("Message folder not found in the specified package.");
                    }

                    if (verbose)
                    {
                        Console.Error.WriteLine("Ensure the package directory has the expected structure and try again.");
                    }
                }

                return;
            }

            // Parse Directory Messages
            if (recursive)
            {
                if (inputPath.Equals(""))
                {
                    Console.Error.WriteLine("Please specify an input directory.");
                    if (verbose)
                    {
                        Console.Error.WriteLine("Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage.");
                    }

                    return;
                }

                if (!IsValidPath(inputPath, true))
                {
                    if (IsValidPath(inputPath, false))
                    {
                        Console.WriteLine("Input path is a file. Processing as a single message.");

                        // Parse Single Message
                        List<string> singleMsgWarnings;
                        if (service)
                        {
                            singleMsgWarnings = ServiceAutoGen.GenerateSingleService(inputPath, outputPath, rosPackageName, verbose);
                        }
                        else if (action)
                        {
                            singleMsgWarnings = ActionAutoGen.GenerateSingleAction(inputPath, outputPath, rosPackageName, verbose);
                        }
                        else
                        {
                            singleMsgWarnings = MessageAutoGen.GenerateSingleMessage(inputPath, outputPath, rosPackageName, verbose);
                        }

                        PrintWarnings(singleMsgWarnings);
                    }
                    else
                    {
                        Console.Error.WriteLine("Invalid input path.");
                    }

                    return;
                }

                Console.WriteLine("Processing directory...");
                List<string> warnings;
                if (service)
                {
                    warnings = ServiceAutoGen.GenerateDirectoryServices(inputPath, outputPath, verbose);
                }
                else if (action)
                {
                    warnings = ActionAutoGen.GenerateDirectoryActions(inputPath, outputPath, verbose);
                }
                else
                {
                    warnings = MessageAutoGen.GenerateDirectoryMessages(inputPath, outputPath, verbose);
                }

                PrintWarnings(warnings);
                return;
            }

            // Unrecognized combination
            Console.Error.WriteLine("Unrecognized combination of arguments. Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' for usage.");
            if (verbose)
            {
                Console.WriteLine("Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage.");
            }
        }

        private static string GetFullPath(string s)
        {
            try
            {
                return Path.GetFullPath(s);
            }
            catch (ArgumentException)
            {
                Console.Error.WriteLine(s + " is an invalid path.");
                return string.Empty;
            }
            catch (SecurityException)
            {
                Console.Error.WriteLine("Permission denied to access: " + s);
                return string.Empty;
            }
            catch (NotSupportedException)
            {
                Console.Error.WriteLine("Path contains an invalid ':' character.");
                return string.Empty;
            }
            catch (PathTooLongException)
            {
                Console.Error.WriteLine("Path, filename, or extension is too long.");
                return string.Empty;
            }
        }

        private static bool IsValidPath(string s, bool isDirectory)
        {
            string path = GetFullPath(s);
            if (string.IsNullOrEmpty(path))
            {
                return false;
            }

            if (isDirectory)
            {
                return Directory.Exists(path);
            }

            return File.Exists(path);
        }

        private static void PrintWarnings(List<string> warnings)
        {
            if (warnings == null)
                return;

            Console.WriteLine("Completed.");
            if (warnings.Count > 0)
            {
                Console.WriteLine("There are " + warnings.Count + " warning(s):");
                foreach (string w in warnings)
                {
                    Console.WriteLine(w);
                }
            }
        }
    }
}
