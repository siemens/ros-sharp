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

using System.Collections.Generic;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public class ActionAutoGen
    {
        public static bool isRos2 = true;
        private static readonly string[] types = {"Goal", "Result", "Feedback"};

        public static List<string> GenerateSingleAction(string inPath, string outPath, string rosPackageName = "", bool verbose = false) {
            // If no ROS package name is provided, extract from path
            if (rosPackageName.Equals(""))
            {
                string[] hierarchy = inPath.Split(new char[] { '/', '\\' });
                rosPackageName = hierarchy[hierarchy.Length - 3];
            }

            outPath = Path.Combine(outPath, MsgAutoGenUtilities.ResolvePackageName(rosPackageName));

            string inFileName = Path.GetFileNameWithoutExtension(inPath);

            if (verbose)
            {
                Console.WriteLine("Parsing: " + inPath);
                Console.WriteLine("Output Location: " + outPath);
            }

            MessageTokenizer tokenizer = new MessageTokenizer(inPath, new HashSet<string>(MsgAutoGenUtilities.builtInTypesMapping.Keys));
            List<List<MessageToken>> listsOfTokens = tokenizer.Tokenize();

            if (listsOfTokens.Count != 3)
            {
                throw new MessageParserException("Unexpected number of sections. Action should have 3 sections.");
            }

            List<string> warnings = new List<string>();

            ActionWrapper actionWrapper = new ActionWrapper(inPath, rosPackageName, outPath);

            for (int i = 0; i < listsOfTokens.Count; i++) {
                List<MessageToken> tokens = listsOfTokens[i];

                // Action is made up of goal, result, feedback
                string className = inFileName + types[i];

                // Parse and generate goal, result, feedback messages
                MessageParser parser = new MessageParser(tokens, outPath, rosPackageName, "action", MsgAutoGenUtilities.builtInTypesMapping, MsgAutoGenUtilities.builtInTypesDefaultInitialValues, className, className, isRos2: isRos2);
                parser.Parse();
                warnings.AddRange(parser.GetWarnings());

                // Generate action section wrapper messages
                actionWrapper.WrapActionSections(types[i]);
            }

            // Generate action wrapper
            actionWrapper.WrapAction();

            return warnings;
        }

        public static List<string> GeneratePackageActions(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            List<string> warnings = new List<string>();

            string[] files = Directory.GetFiles(Path.Combine(inPath, "action"), "*.action");

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No action files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " action files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleAction(file, outPath, rosPackageName, verbose));
                }
            }

            return warnings;
        }

        public static List<string> GenerateDirectoryActions(string inPath, string outPath, bool verbose = false)
        {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            string[] files = Directory.GetFiles(inPath, "*.action", SearchOption.AllDirectories);

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No action files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " action files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleAction(file, outPath, verbose: verbose));
                }
            }
            return warnings;
        }
    }

    public class ActionWrapper {

        private const string ONE_TAB = "    ";
        private const string TWO_TABS = "        ";

        private readonly string inPath;
        private readonly string inFileName;

        private readonly string rosPackageName;

        private readonly string outPath;

        private Dictionary<string, string> symbolTable;

        public ActionWrapper(string inPath, string rosPackageName, string outPath) {
            this.inPath = inPath;
            this.inFileName = Path.GetFileNameWithoutExtension(inPath);
            this.rosPackageName = rosPackageName;
            this.outPath = Path.Combine(outPath, "action");
        }

        private string GenerateDefaultValueConstructor(string className)
        {
            string constructor = "";

            constructor += TWO_TABS + "public " + className + "() : base()\n";
            constructor += TWO_TABS + "{\n";

            foreach (string identifier in symbolTable.Keys)
            {
                string type = symbolTable[identifier];

                string adjustedIdentifier = identifier;

                if (ActionAutoGen.isRos2)
                {
                    if (identifier.Equals("goal"))
                    {
                        adjustedIdentifier = "args";
                    }
                    else if (identifier.Equals("result") || identifier.Equals("feedback"))
                    {
                        adjustedIdentifier = "values";
                    }
                    else
                    {
                        adjustedIdentifier = identifier;
                        Console.WriteLine("Warning: Unrecognized identifier: " + identifier);
                    }
                }

                constructor += TWO_TABS + ONE_TAB + "this." + adjustedIdentifier + " = ";
                constructor += "new " + type + "();\n";
            }

            constructor += TWO_TABS + "}\n";

            return constructor;
        }

        private string GenerateParameterizedConstructor(string className, string msgType)
        {
            string constructor = "";

            string paramsIn = "";
            string paramsOut = "";
            string assignments = "";

            if (ActionAutoGen.isRos2)
            {
                if (msgType.Equals("Goal")) 
                {
                    paramsIn += "Header header, GoalInfo goalInfo, ";
                    paramsOut += "header, goalInfo";
                }
                else if (msgType.Equals("Result"))
                {
                    paramsIn += "Header header, string action, sbyte status, bool result, string id, ";
                    paramsOut += "header, action, status, result, id";
                }
                else if (msgType.Equals("Feedback"))
                {
                    paramsIn += "Header header, string id, string action, ";
                    paramsOut += "header, id, action";
                }
                else
                {
                    Console.WriteLine("Warning: Unrecognized message type: " + msgType);
                }
            }

            else
            {
                if (msgType.Equals("Goal"))
                {
                    paramsIn += "Header header, GoalID goal_id, ";
                    paramsOut += "header, goal_id";
                }
                else if (msgType.Equals("Result") || msgType.Equals("Feedback"))
                {
                    paramsIn += "Header header, GoalStatus status, ";
                    paramsOut += "header, status";
                }
            }

            //if (msgType.Equals("Goal"))
            //{
            //    paramsIn += "Header header, GoalID goal_id, ";
            //    paramsOut += "header, goal_id";
            //}
            //else if (msgType.Equals("Result") || msgType.Equals("Feedback")) {
            //    paramsIn += "Header header, GoalStatus status, ";
            //    paramsOut += "header, status";
            //}

            foreach (string identifier in symbolTable.Keys)
            {
                string type = symbolTable[identifier];

                string adjustedIdentifier = identifier;

                if (ActionAutoGen.isRos2)
                {
                    if (identifier.Equals("goal"))
                    {
                        adjustedIdentifier = "args";
                    }
                    else if (identifier.Equals("result") || identifier.Equals("feedback"))
                    {
                        adjustedIdentifier = "values";
                    }
                    else
                    {
                        adjustedIdentifier = identifier;
                        Console.WriteLine("Warning: Unrecognized identifier: " + identifier);
                    }
                }

                paramsIn += type + " " + adjustedIdentifier + ", ";
                assignments += TWO_TABS + ONE_TAB + "this." + adjustedIdentifier + " = " + adjustedIdentifier + ";\n";
            }

            if (!paramsIn.Equals(""))
            {
                paramsIn = paramsIn.Substring(0, paramsIn.Length - 2);
            }

            constructor += TWO_TABS + "public " + className + "(" + paramsIn + ") : base(" + paramsOut + ")\n";
            constructor += TWO_TABS + "{\n";
            constructor += assignments;
            constructor += TWO_TABS + "}\n";

            return constructor;
        }

        public void WrapActionSections(string type)
        {
            string wrapperName = inFileName + "Action" + type;
            string msgName = inFileName + type;

            string outPath = Path.Combine(this.outPath, wrapperName + ".cs");

            string imports = "using RosSharp.RosBridgeClient.MessageTypes.Std;\n";

            if (ActionAutoGen.isRos2)
            {
                imports += "using RosSharp.RosBridgeClient.MessageTypes.Action;\n\n";
            }
            else
            {
                imports += "using RosSharp.RosBridgeClient.MessageTypes.Actionlib;\n\n";
            }

            symbolTable = new Dictionary<string, string>();

            using (StreamWriter writer = new StreamWriter(outPath, false))
            {
                // Write block comment
                writer.Write(MsgAutoGenUtilities.BLOCK_COMMENT + "\n");

                // Write preprocessor directive: Begin
                if (ActionAutoGen.isRos2)
                {
                    writer.Write("#if ROS2\n");
                }
                else
                {
                    writer.Write("#if !ROS2\n");
                }

                // Write imports
                writer.Write(imports);

                // Write namespace
                writer.Write(
                    "namespace RosSharp.RosBridgeClient.MessageTypes." + MsgAutoGenUtilities.ResolvePackageName(rosPackageName) + "\n" +
                    "{\n"
                    );

                // Write class declaration
                writer.Write(
                    ONE_TAB + "public class " + wrapperName + " : Action" + type + "<" + inFileName + type +  ">\n" +
                    ONE_TAB + "{\n"
                    );

                // Write ROS package name
                //writer.Write(
                //    TWO_TABS + "public const string RosMessageName = \"" + rosPackageName + "/" + wrapperName + "\";\n"
                //    );

                if (ActionAutoGen.isRos2)
                {
                    writer.Write(
                        TWO_TABS + "public const string RosMessageName = \"" + rosPackageName + "/" + "action" + "/"
                        + wrapperName + "\";\n");
                }
                else
                {
                    writer.Write(
                        TWO_TABS + "public const string RosMessageName = \"" + rosPackageName + "/" + wrapperName + "\";\n"
                        );
                }
                Console.WriteLine("RosMessageName: " + rosPackageName + "/" + wrapperName);

                // Record goal/result/feedback declaration
                symbolTable.Add(MsgAutoGenUtilities.LowerFirstLetter(type), msgName);

                writer.Write("\n");

                // Write default value constructor
                writer.Write(GenerateDefaultValueConstructor(wrapperName) + "\n");

                // Write parameterized constructor
                writer.Write(GenerateParameterizedConstructor(wrapperName, type));

                // Close class
                writer.Write(ONE_TAB + "}\n");
                // Close namespace
                writer.Write("}\n");

                // Write preprocessor directive: End
                writer.Write("#endif\n");
            }
        }

        public void WrapAction()
        {
            string wrapperName = inFileName + "Action";

            string outPath = Path.Combine(this.outPath, wrapperName + ".cs");

            string imports = "\n\n";

            symbolTable = new Dictionary<string, string>();

            using (StreamWriter writer = new StreamWriter(outPath, false))
            {
                // Write block comment
                writer.Write(MsgAutoGenUtilities.BLOCK_COMMENT + "\n");

                // Write preprocessor directive: Begin
                if (ActionAutoGen.isRos2)
                {
                    writer.Write("#if ROS2\n");
                }
                else
                {
                    writer.Write("#if !ROS2\n");
                }

                // Write imports
                writer.Write(imports);

                // Write namespace
                writer.Write(
                    "namespace RosSharp.RosBridgeClient.MessageTypes." + MsgAutoGenUtilities.ResolvePackageName(rosPackageName) + "\n" +
                    "{\n"
                    );

                // Write class declaration
                string[] genericParams = new string[] {
                    inFileName + "ActionGoal",
                    inFileName + "ActionResult",
                    inFileName + "ActionFeedback",
                    inFileName + "Goal",
                    inFileName + "Result",
                    inFileName + "Feedback"
                };
                writer.Write(
                    ONE_TAB + "public class " + wrapperName + " : Action<" + string.Join(", ", genericParams) +  ">\n" +
                    ONE_TAB + "{\n"
                    );

                // Write ROS package name
                if (ActionAutoGen.isRos2)
                {
                    writer.Write(
                        TWO_TABS + "public const string RosMessageName = \"" + rosPackageName + "/" + "action" + "/" 
                        + wrapperName + "\";\n");
                }
                else
                {
                    writer.Write(
                        TWO_TABS + "public const string RosMessageName = \"" + rosPackageName + "/" + wrapperName + "\";\n"
                        );
                }

                // Record variables
                // Action Goal
                symbolTable.Add("action_goal", wrapperName + "Goal");
                // Action Result
                symbolTable.Add("action_result", wrapperName + "Result");
                //Action Feedback
                symbolTable.Add("action_feedback", wrapperName + "Feedback");

                // Write default value constructor
                writer.Write("\n" + GenerateDefaultValueConstructor(wrapperName) + "\n");

                // Close class
                writer.Write(ONE_TAB + "}\n");
                // Close namespace
                writer.Write("}\n");

                // Write preprocessor directive: End
                writer.Write("#endif\n");
            }
        }

    }
}

