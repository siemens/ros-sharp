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

namespace RosBridgeClient.Messages
{
    public static class ServiceMessageGenerator
    {
        public static void Generate(string serviceName, string rosPackageName, MessageElement[] requestElements, MessageElement[] responseElements, string assetPath)
        {
            using (StreamWriter outfile = new StreamWriter(assetPath + "/" + serviceName + ".cs", false))
            {
                outfile.WriteLine("/*");
                outfile.WriteLine("This message class is generated automatically with 'ServiceMessageGenerator' of ROS#");
                outfile.WriteLine("*/ \n");

                outfile.WriteLine(
                    "using Newtonsoft.Json;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Geometry;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Navigation;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Sensor;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Standard;\n" +
                    "using RosSharp.RosBridgeClient.Messages.Actionlib;\n\n" +

                    "namespace RosSharp.RosBridgeClient.Services\n" +
                    "{\n" +

                #region ServiceRequest
                        "public class " + serviceName + "Request" + " : Message\n" +
                        "{\n" +
                            "[JsonIgnore]\n" +
                            "public const string RosMessageName = \"" + rosPackageName + "/" + serviceName + "\";\n");

                for (int i = 0; i < requestElements.Length; i++)
                    outfile.WriteLine(requestElements[i].getDeclerationString());

                outfile.Write("\npublic " + serviceName + "Request" + "(");
                for (int i = 0; i < requestElements.Length-1; i++)
                    outfile.Write(requestElements[i].messageType + " _" + requestElements[i].messageName + ", ");
                outfile.Write(requestElements[requestElements.Length-1].messageType + " _" + requestElements[requestElements.Length-1].messageName + ")");


                outfile.Write("{");

                for (int i = 0; i < requestElements.Length; i++)
                    outfile.WriteLine(requestElements[i].messageName + " = _" + requestElements[i].messageName + ";");

                outfile.WriteLine(
                             "}\n" +

                        "}\n");

                #endregion ServiceRequest

                #region ServiceResponse
                outfile.WriteLine(
                          "public class " + serviceName + "Response" + " : Message\n" +
                        "{\n" +
                            "[JsonIgnore]\n" +
                            "public const string RosMessageName = \"" + rosPackageName + "/" + serviceName + "\";\n");

                for (int i = 0; i < responseElements.Length; i++)
                    outfile.WriteLine(responseElements[i].getDeclerationString());

                outfile.WriteLine(
                          "}\n" +
                      "}\n");
                #endregion ServiceResponse

            }
        }
    }
}
