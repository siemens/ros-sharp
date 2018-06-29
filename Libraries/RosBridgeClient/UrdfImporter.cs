/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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
using System.Threading;
using System.Linq;
using System.Xml.Linq;

using file_server = RosSharp.RosBridgeClient.Services.FileServer;
using rosapi = RosSharp.RosBridgeClient.Services.RosApi;

namespace RosSharp.RosBridgeClient
{
    public delegate void ReceiveEventHandler<Tin, Tout>(ServiceReceiver<Tin, Tout> sender, Tout ServiceResponse) where Tin : Message where Tout : Message;

    public class ServiceReceiver<Tin,Tout> where Tin: Message where Tout:Message
    {
        public string Service { get; private set; }
        public Tin ServiceParameter { get; private set; }
        public object HandlerParameter { get; set; }
        public event ReceiveEventHandler<Tin, Tout> ReceiveEventHandler;

        public ServiceReceiver(RosSocket rosSocket, string service, Tin parameter, object handlerParameter)
        {
            Service = service;
            ServiceParameter = parameter;
            HandlerParameter = handlerParameter;
            rosSocket.CallService<Tin,Tout>(Service, Receive, ServiceParameter);
        }
        private void Receive(Tout ServiceResponse)
        {
            if (ReceiveEventHandler != null)
                ReceiveEventHandler.Invoke(this, ServiceResponse);
        }
    }

    public class UrdfImporter
    {
        private RosSocket rosSocket;
        private string localDirectory;
        public string RobotName { get; private set; }

        public string LocalDirectory
        {
            get
            {
                Status["robotNameReceived"].WaitOne();
                return Path.Combine(localDirectory, RobotName);
            }
        }

        public Dictionary<string, ManualResetEvent> Status = new Dictionary<string, ManualResetEvent>
        {
            { "robotNameReceived",new ManualResetEvent(false) },
            { "robotDescriptionReceived", new ManualResetEvent(false) },
            { "resourceFilesReceived", new ManualResetEvent(false) }
        };

        public Dictionary<Uri, bool> RequestedResourceFiles = new Dictionary<Uri, bool>();

        public UrdfImporter(RosSocket _rosSocket, string _localDirectory)
        {
            rosSocket = _rosSocket;
            localDirectory = _localDirectory;
        }

        public bool Import(int maxTimeOut = int.MaxValue)
        {
            rosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>("/rosapi/get_param",
                                                                                    ReceiveRobotName,
                                                                                    new rosapi.GetParamRequest("/robot/name", "default"));

            var robotDescriptionReceiver = new ServiceReceiver<rosapi.GetParamRequest,  rosapi.GetParamResponse>(rosSocket, "/rosapi/get_param",
                                                                                        new rosapi.GetParamRequest("/robot_description", "default"),
                                                                                        Path.DirectorySeparatorChar + "robot_description.urdf");

            robotDescriptionReceiver.ReceiveEventHandler += ReceiveRobotDescription;

            return (WaitHandle.WaitAll(Status.Values.ToArray(), maxTimeOut));
        }

        private void ReceiveRobotName(object serviceResponse)
        {
            RobotName = FormatTextFileContents(((rosapi.GetParamResponse)serviceResponse).value);
            Status["robotNameReceived"].Set();
        }

        private void ReceiveRobotDescription(ServiceReceiver<rosapi.GetParamRequest, rosapi.GetParamResponse> serviceReciever, rosapi.GetParamResponse serviceResponse)
        {
            string robotDescription = FormatTextFileContents(serviceResponse.value);

            Thread importResourceFilesThread = new Thread(() => ImportResourceFiles(robotDescription));
            importResourceFilesThread.Start();

            Thread writeTextFileThread = new Thread(() => WriteTextFile((string)serviceReciever.HandlerParameter, robotDescription));
            writeTextFileThread.Start();

            Status["robotDescriptionReceived"].Set();
        }

        private void ImportResourceFiles(string fileContents)
        {
            var serviceReceivers = RequestResourceFiles(ReadResourceFileUris(fileContents));
            if (serviceReceivers.Count == 0)
            {
                Status["resourceFilesReceived"].Set();
                return;
            }

            foreach (var serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += ReceiveResourceFile;

        }

        private static List<Uri> ReadResourceFileUris(string robotDescription)
        {
            XElement root = XElement.Parse(robotDescription);
            return (from seg in root.Descendants("mesh") where seg.Attribute("filename") != null select new Uri(seg.Attribute("filename").Value)).ToList();
        }

        private List<ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>> RequestResourceFiles(List<Uri> resourceFileUris)
        {
            var serviceReceivers = new List<ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>>();
            foreach (Uri resourceFilePath in resourceFileUris)
            {
                if (!RequestedResourceFiles.ContainsKey(resourceFilePath))
                {
                    RequestedResourceFiles.Add(resourceFilePath, false);
                    serviceReceivers.Add(new ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>(rosSocket, "/file_server/get_file", new file_server.GetBinaryFileRequest(resourceFilePath.ToString()), GetLocalFilename(resourceFilePath)));
                }
            }
            return serviceReceivers;
        }

        private void ReceiveResourceFile(ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse> serviceReceiver, file_server.GetBinaryFileResponse serviceResponse)
        {
            byte[] fileContents = (serviceResponse).value;
            Uri resourceFileUri = new Uri((serviceReceiver.ServiceParameter).name);

            if (IsColladaFile(resourceFileUri))
            {
                Thread importResourceFilesThread = new Thread(() => ImportColladaTextureFiles(resourceFileUri, System.Text.Encoding.UTF8.GetString(fileContents)));
                importResourceFilesThread.Start();
            }
            Thread writeTextFileThread = new Thread(() => WriteBinaryResponseToFile((string)serviceReceiver.HandlerParameter, fileContents));
            writeTextFileThread.Start();

            UpdateFileRequestStatus(resourceFileUri);
        }

        private void UpdateFileRequestStatus(Uri resourceFileUri)
        {
            RequestedResourceFiles[resourceFileUri] = true;
            if (RequestedResourceFiles.Values.All(x => x == true))
                Status["resourceFilesReceived"].Set();
        }

        private static bool IsColladaFile(Uri uri)
        {
            return Path.GetExtension(uri.LocalPath) == ".dae";
        }
        private void ImportColladaTextureFiles(Uri daeFileUri, string fileContents)
        {
            var serviceReceivers = RequestResourceFiles(ReadDaeTextureUris(daeFileUri, fileContents));
            foreach (var serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += ReceiveTextureFiles;
        }

        private List<Uri> ReadDaeTextureUris(Uri resourceFileUri, string fileContents)
        {
            XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
            XElement root = XElement.Parse(fileContents);
            return (from x in root.Elements()
                    where x.Name.LocalName == "library_images"
                    select new Uri(resourceFileUri, x.Element(xmlns + "image").Element(xmlns + "init_from").Value)).ToList();
        }

        private void ReceiveTextureFiles(ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse> serviceReceiver, file_server.GetBinaryFileResponse serviceResponse)
        {
            WriteBinaryResponseToFile((string)serviceReceiver.HandlerParameter, serviceResponse.value);
            UpdateFileRequestStatus(new Uri(serviceReceiver.ServiceParameter.name));
        }

        private void WriteBinaryResponseToFile(string relativeLocalFilename, byte[] fileContents)
        {
            string filename = LocalDirectory + relativeLocalFilename;
            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllBytes(filename, fileContents);
        }

        private void WriteTextFile(string relativeLocalFilename, string fileContents)
        {
            string filename = LocalDirectory + relativeLocalFilename;
            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllText(filename, fileContents);
        }

        private static string GetLocalFilename(Uri resourceFilePath)
        {
            return Path.DirectorySeparatorChar
                + resourceFilePath.Host
                + resourceFilePath.LocalPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
        }

        private static string FormatTextFileContents(string fileContents)
        {
            // remove enclosing quotations if existend:
            if (fileContents.Substring(0, 1) == "\"" && fileContents.Substring(fileContents.Length - 1, 1) == "\"")
                fileContents = fileContents.Substring(1, fileContents.Length - 2);

            // replace \" quotation sign by actual quotation:
            fileContents = fileContents.Replace("\\\"", "\"");

            // replace \n newline sign by actual new line:
            return fileContents.Replace("\\n", Environment.NewLine);
        }
    }
}
