/*
© Siemens AG, 2019
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosBridgeClient.Messages
{

    public class SimpleMessageGenerator : MonoBehaviour
    {
 
        public string MessageName;
        public string RosPackageName;
        public MessageComponent[] MessageComponents;

        public void Generate()
        {
            Generate(MessageName, RosPackageName, MessageComponents);
        }

        public static void Generate(string messageName, string rosPackageName, MessageComponent[] messageComponents)
        {
            string copyPath = "Assets/RosSharp/Scripts/RosbridgeClient/MessageClass/" + messageName + ".cs";

            using (StreamWriter outfile = new StreamWriter(copyPath, false))
            {
                outfile.WriteLine("/*");
                outfile.WriteLine("This message class is generated automatically with 'SimpleMessageGenerator' of ROS#");
                outfile.WriteLine("*/ \n");


                outfile.WriteLine(
                    "using Newtonsoft.Json;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Geometry;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Navigation;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Sensor;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Standard;\n" + 
                    "using RosSharp.RosBridgeClient.Messages.Actionlib;\n\n" + 

                    
                    "namespace RosSharp.RosBridgeClient.Messages\n" +
                    "{\n" +

                        "public class " + messageName + " : Message\n" +
                        "{\n" +
                            "[JsonIgnore]\n" +
                            "public const string RosMessageName = \"" + rosPackageName + "/" + messageName + "\";\n");

                for (int i = 0; i < messageComponents.Length; i++)
                    outfile.WriteLine(messageComponents[i].getDeclerationString());


                outfile.WriteLine(
                            "\npublic " + messageName + "()\n" +
                             "{");

                for (int i = 0; i < messageComponents.Length; i++)
                    outfile.WriteLine(messageComponents[i].getDefinitionString());

                outfile.WriteLine(
                             "}\n" +

                        "}\n" +
                    "}\n");

            }
            AssetDatabase.Refresh();
        }


    }
}
