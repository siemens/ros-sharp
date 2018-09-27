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

namespace RosSharp.RosBridgeClient.UrdfTransfer
{
    public class UrdfTransferFromRos : UrdfTransfer
    {
        private readonly string localUrdfDirectory;
        
        public string LocalUrdfDirectory
        {
            get
            {
                Status["robotNameReceived"].WaitOne();
                return Path.Combine(localUrdfDirectory, RobotName);
            }
        }

        public UrdfTransferFromRos(RosSocket rosSocket, string localUrdfDirectory)
        {
            RosSocket = rosSocket;
            this.localUrdfDirectory = localUrdfDirectory;

            Status = new Dictionary<string, ManualResetEvent>
            {
                {"robotNameReceived", new ManualResetEvent(false)},
                {"robotDescriptionReceived", new ManualResetEvent(false)},
                {"resourceFilesReceived", new ManualResetEvent(false)}
            };

            FilesBeingProcessed = new Dictionary<string, bool>();
        }

        public override void Transfer()
        {
            RosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>("/rosapi/get_param",
                                                                                    ReceiveRobotName,
                                                                                    new rosapi.GetParamRequest("/robot/name", "default"));

            var robotDescriptionReceiver = new ServiceReceiver<rosapi.GetParamRequest,  rosapi.GetParamResponse>(RosSocket, "/rosapi/get_param",
                                                                                        new rosapi.GetParamRequest("/robot_description", "default"),
                                                                                        Path.DirectorySeparatorChar + "robot_description.urdf");

            robotDescriptionReceiver.ReceiveEventHandler += ReceiveRobotDescription;  
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
            List<Uri> resourceFileUris = ReadResourceFileUris(XDocument.Parse(fileContents));
            var serviceReceivers = RequestResourceFiles(resourceFileUris);
            if (serviceReceivers.Count == 0)
            {
                Status["resourceFilesReceived"].Set();
                return;
            }

            foreach (var serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += ReceiveResourceFile;
        }

        private List<ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>> RequestResourceFiles(List<Uri> resourceFileUris)
        {
            var serviceReceivers = new List<ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>>();
            foreach (Uri resourceFilePath in resourceFileUris)
            {
                if (!FilesBeingProcessed.ContainsKey(resourceFilePath.ToString()))
                {
                    FilesBeingProcessed.Add(resourceFilePath.ToString(), false);
                    serviceReceivers.Add(
                        new ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>(
                            RosSocket,
                            "/file_server/get_file",
                            new file_server.GetBinaryFileRequest(resourceFilePath.ToString()),
                            GetLocalFilename(resourceFilePath)));
                }
            }
            return serviceReceivers;
        }

        private void ReceiveResourceFile(ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse> serviceReceiver, file_server.GetBinaryFileResponse serviceResponse)
        {
            byte[] fileContents = serviceResponse.value;
            Uri resourceFileUri = new Uri((serviceReceiver.ServiceParameter).name);

            if (IsColladaFile(resourceFileUri))
            {
                Thread importResourceFilesThread = new Thread(() => ImportColladaTextureFiles(resourceFileUri, System.Text.Encoding.UTF8.GetString(fileContents)));
                importResourceFilesThread.Start();
            }
            Thread writeTextFileThread = new Thread(() => WriteBinaryResponseToFile((string)serviceReceiver.HandlerParameter, fileContents));
            writeTextFileThread.Start();

            UpdateFileRequestStatus(resourceFileUri.ToString());
        }

        private void UpdateFileRequestStatus(string resourceFilePath)
        {
            FilesBeingProcessed[resourceFilePath] = true;
            if (FilesBeingProcessed.Values.All(x => x == true))
                Status["resourceFilesReceived"].Set();
        }
        
        private void ImportColladaTextureFiles(Uri daeFileUri, string fileContents)
        {
            XDocument xDocument = XDocument.Parse(fileContents);
            var serviceReceivers = RequestResourceFiles(ReadDaeTextureUris(daeFileUri, xDocument));
            foreach (var serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += ReceiveTextureFiles;
        }

        private void ReceiveTextureFiles(ServiceReceiver<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse> serviceReceiver, file_server.GetBinaryFileResponse serviceResponse)
        {
            WriteBinaryResponseToFile((string)serviceReceiver.HandlerParameter, serviceResponse.value);
            UpdateFileRequestStatus(serviceReceiver.ServiceParameter.name);
        }

        private void WriteBinaryResponseToFile(string relativeLocalFilename, byte[] fileContents)
        {
            string filename = LocalUrdfDirectory + relativeLocalFilename;
            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllBytes(filename, fileContents);
        }

        private void WriteTextFile(string relativeLocalFilename, string fileContents)
        {
            string filename = LocalUrdfDirectory + relativeLocalFilename;
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

    public delegate void ReceiveEventHandler<Tin, Tout>(ServiceReceiver<Tin, Tout> sender, Tout ServiceResponse) where Tin : Message where Tout : Message;

    public class ServiceReceiver<Tin, Tout> where Tin : Message where Tout : Message
    {
        public readonly Tin ServiceParameter;
        public readonly object HandlerParameter;
        public event ReceiveEventHandler<Tin, Tout> ReceiveEventHandler;

        public ServiceReceiver(RosSocket rosSocket, string service, Tin parameter, object handlerParameter)
        {
            ServiceParameter = parameter;
            HandlerParameter = handlerParameter;
            rosSocket.CallService<Tin, Tout>(service, Receive, ServiceParameter);
        }
        private void Receive(Tout serviceResponse)
        {
            ReceiveEventHandler?.Invoke(this, serviceResponse);
        }
    }
}
