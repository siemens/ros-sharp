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
    public class ServiceAutoGen
    {
        public static bool isRos2 = true;
        private static readonly string[] types = { "Request", "Response" };

        public static List<string> GenerateSingleService(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
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

            if (listsOfTokens.Count != 2)
            {
                throw new MessageParserException("Unexpected number of sections. Service should have 2 sections.");
            }

            List<string> warnings = new List<string>();

            for (int i = 0; i < listsOfTokens.Count; i++)
            {
                List<MessageToken> tokens = listsOfTokens[i];

                // Service is made up of request and response
                string className = inFileName + types[i];

                MessageParser parser = new MessageParser(tokens, outPath, rosPackageName, "srv", MsgAutoGenUtilities.builtInTypesMapping, MsgAutoGenUtilities.builtInTypesDefaultInitialValues, className, isRos2: isRos2);
                parser.Parse();
                warnings.AddRange(parser.warnings);
            }
            return warnings;
        }

        public static List<string> GeneratePackageServices(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            List<string> warnings = new List<string>();

            string[] files = Directory.GetFiles(Path.Combine(inPath, "srv"), "*.srv");

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No service files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " service files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleService(file, outPath, rosPackageName, verbose));
                }
            }

            return warnings;
        }

        public static List<string> GenerateDirectoryServices(string inPath, string outPath, bool verbose = false)
        {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            string[] files = Directory.GetFiles(inPath, "*.srv", SearchOption.AllDirectories);

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No service files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " service files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleService(file, outPath, verbose: verbose));
                }
            }
            return warnings;
        }
    }
}
