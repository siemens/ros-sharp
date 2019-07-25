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
using System.IO;
using System.Security;

using System.Collections.Generic;

using RosSharp.RosBridgeClient.MessageGeneration;

namespace RosSharp.RosBridgeClient.MessageGenerationConsoleTool
{
    class RosMsgGen
    {
        private static readonly string usage =
            "Usage:\n" +
            "RosMsgGen.exe [-h | --help] [-v | --verbose] [-s | --service] [-a | --action] [-r | --recursive] [-p | --package] <input-path> [-n | --ros-package-name <package-name>] [-o | --output <output-path>]\n" +
            "    help\t\t\tPrints this message. Only valid if it is the first flag\n" +
            "    verbose\t\t\tOutputs extra information\n" +
            "    service\t\t\tGenerate service messages\n" +
            "    action\t\t\tGenerate action messages\n" +
            "    recursive\t\t\tGenerate message for all ROS message files in the specified directory\n" +
            "    package\t\t\tTreat the directory as a single ROS package during message generation\n" +
            "    ros-package-name\t\tSpecify the ROS package name for the message\n" +
            "                    \t\tIf unspecified, package name will be retrieved from path, assuming ROS package structure\n" +
            "    output\t\t\tSpecify output path\n" +
            "          \t\t\tIf unspecified, output will be in current working directory, under RosSharpMessages\n\n" +
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
            "-o", "--output"
        };

        private static readonly string defaultOutputDirectory = Path.Combine(Directory.GetCurrentDirectory(), "RosSharpMessages");

        public static void Main(string[] args)
        {
            // Arguments
            string inputPath = "";

            bool verbose = false;

            bool service = false;
            bool action = false;

            bool recursive = false;
            bool package = false;
            string rosPackageName = "";
            string outputPath = defaultOutputDirectory;

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

            for (int i = 0; i < args.Length; i++) {
                string arg = args[i];

                if (arg.Equals("-s") || arg.Equals("--service")) {
                    if (action) {
                        Console.Error.WriteLine("Option '--service' conflict with '--action'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Make up your mind!");
                        }
                        return;
                    }
                    service = true;
                    if (verbose) {
                        Console.WriteLine("'--service' flag seen. Expecting *.srv input");
                    }
                    continue;
                }

                if (arg.Equals("-a") || arg.Equals("--action"))
                {
                    if (service)
                    {
                        Console.Error.WriteLine("Option '--action' conflict with '--service'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Make up your mind!");
                        }
                        return;
                    }
                    action = true;
                    if (verbose)
                    {
                        Console.WriteLine("'--action' flag seen. Expecting *.action input");
                    }
                    continue;
                }

                if (arg.Equals("-r") || arg.Equals("--recursive")) {
                    if (package)
                    {
                        Console.Error.WriteLine("Option '--recursive' conflict with '--package'.");
                        if (verbose) {
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
                    if (!rosPackageName.Equals("")) {
                        Console.WriteLine("'--recursive' option enabled, ignoring ROS package name");
                    }
                    recursive = true;
                    continue;
                }

                if (arg.Equals("-p") || arg.Equals("--package")) {
                    if (recursive) {
                        Console.Error.WriteLine("Option '--package' conflict with '--recursive'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("Make up your mind!");
                        }
                        return;
                    }
                    if (package) {
                        Console.WriteLine("Duplicate flag '--package'.");
                        if (verbose)
                        {
                            Console.Error.WriteLine("I got it already!");
                        }
                    }
                    package = true;
                    continue;
                }

                if (arg.Equals("-n") || arg.Equals("--ros-package-name")) {
                    if (i == args.Length - 1)
                    {
                        Console.Error.WriteLine("Missing ROS package name. Will get name from path");
                    }
                    else if (validOptions.Contains(args[i+1])) {
                        Console.Error.WriteLine("Missing ROS package name. Will get name from path");
                    }
                    else if (recursive) {
                        Console.WriteLine("'--recursive' option enabled, ignoring ROS package name");
                    }
                    else
                    {
                        rosPackageName = args[i + 1];
                        i++;
                    }
                    continue;
                }

                if (arg.Equals("-o") || arg.Equals("--output")) {
                    if (i == args.Length - 1)
                    {
                        Console.Error.WriteLine("Missing output path. Using default output path.");
                    }
                    else if (validOptions.Contains(args[i + 1]))
                    {
                        Console.Error.WriteLine("Missing output path. Using default output path.");
                    }
                    else {
                        outputPath = args[i + 1];
                        outputPath = Path.Combine(outputPath, "RosSharpMessages");
                        i++;
                    }
                    continue;
                }

                if (arg.Equals("-v") || arg.Equals("--verbose")) {
                    Console.WriteLine("Verbose flag detected. Warning: A huge wave of text incoming!");
                    verbose = true;
                    continue;
                }

                if (inputPath.Equals(""))
                {
                    inputPath = arg;
                }
                else {
                    Console.Error.WriteLine("Ignored invalid argument: " + arg);
                    if (verbose)
                    {
                        Console.Error.WriteLine("Mumble mumble. Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage");
                    }
                }
            }

            // Do work
            // Parse Individual Messages
            if (!package && !recursive)
            {
                if (inputPath.Equals(""))
                {
                    Console.Error.WriteLine("Please specify input file");
                    if (verbose)
                    {
                        Console.Error.WriteLine("Bricks without straw");
                    }
                    return;
                }
                if (!IsValidPath(inputPath, false)) {
                    if (IsValidPath(inputPath, true))
                    {
                        Console.Error.WriteLine("Input path is a directory. Please use --package or --recursive option");
                    }
                    else {
                        Console.Error.WriteLine("Invalid input path");
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
            if (package) {
                if (inputPath.Equals(""))
                {
                    Console.Error.WriteLine("Please specify input package");
                    if (verbose)
                    {
                        Console.Error.WriteLine("Bricks without straw");
                    }
                    return;
                }
                if (!IsValidPath(inputPath, true))
                {
                    if (IsValidPath(inputPath, false))
                    {
                        Console.Error.WriteLine("Input path is a file. Please drop --package option");
                    }
                    else
                    {
                        Console.Error.WriteLine("Invalid input path");
                    }
                    return;
                }
                try
                {
                    Console.WriteLine("Working...");
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
                catch (DirectoryNotFoundException) {
                    if (service)
                    {
                        Console.Error.WriteLine("Didn't find service folder in given package");
                    }
                    else
                    {
                        Console.Error.WriteLine("Didn't find message folder in given package");
                    }
                    if (verbose)
                    {
                        Console.Error.WriteLine("Bricks without straw");
                    }
                }
                return;
            }
            // Parse Directory Message
            if (recursive) {
                if (inputPath.Equals(""))
                {
                    Console.Error.WriteLine("Please specify input directory");
                    if (verbose)
                    {
                        Console.Error.WriteLine("Bricks without straw");
                    }
                    return;
                }
                if (!IsValidPath(inputPath, true))
                {
                    if (IsValidPath(inputPath, false))
                    {
                        Console.Error.WriteLine("Input path is a file. Consider as base case");
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
                        Console.Error.WriteLine("Invalid input path");
                    }
                    return;
                }
                Console.WriteLine("Working...Checkout xkcd.com/303");
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

            // Otherwise I don't know...
            Console.Error.WriteLine("Unrecognized combination of arguments.");
            if (verbose)
            {
                Console.WriteLine("Mumble Mumble. Run 'RosMsgGen.exe -h' or 'RosMsgGen.exe --help' to see usage");
            }
        }

        private static string GetFullPath(string s) {
            try
            {
                return Path.GetFullPath(s);
            }
            catch (ArgumentException)
            {
                Console.Error.WriteLine(s + " is an invalid path");
                return "";
            }
            catch (SecurityException)
            {
                Console.Error.WriteLine("Permission Denied to " + s);
                return "";
            }
            catch (NotSupportedException)
            {
                Console.Error.WriteLine("Path cannot contain a ':' that is not part of a volume identifier");
                return "";
            }
            catch (PathTooLongException) {
                Console.Error.WriteLine("Path, filename or extension is too long");
                return "";
            }
        }

        private static bool IsValidPath(string s, bool isDirectory) {
            string path = GetFullPath(s);
            if (path.Equals(""))
            {
                return false;
            }
            else {
                if (isDirectory)
                {
                    return Directory.Exists(path);
                }
                else {
                    return File.Exists(path);
                }
            }
        }

        private static void PrintWarnings(List<string> warnings) {
            if (warnings != null) {
                Console.WriteLine("Done.");
                if (warnings.Count > 0)
                {
                    Console.WriteLine("You have " + warnings.Count + " warnings");
                    foreach (string w in warnings)
                    {
                        Console.WriteLine(w);
                    }
                }
            }
        }
    }
}
