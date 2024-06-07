/*
© Siemens AG, 2017-2019
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

* Removed additional RosConnector instance. Urdf transfer is now handled by the existing RosConnector component.
* If no RosConnector component is present in the scene, one can be created by pressing the button.
* RosConnector specific input fields have been removed as they are no longer required.
* Robot name parameter input field added.
* The 'Reset to Default' button now behaves according to the selected ROS version (from the RosConnector component). 
* Added GUI hints for parameter syntax. 
    (C) Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

using System;
using System.IO;
using System.Collections.Generic;
using System.Threading;
using System.Linq;
using System.Xml.Linq;
using file_server = RosSharp.RosBridgeClient.MessageTypes.FileServer;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;

namespace RosSharp.RosBridgeClient.UrdfTransfer
{
    public class UrdfTransferFromRos : UrdfTransfer
    {
#if !ROS2
        private const string DEFAULT_STRING = "default";
#else
        private const string DEFAULT_STRING = "default_value";
#endif

        private readonly string localUrdfDirectory;
        private string urdfParameter;
        private string robotNameParameter;

        public string LocalUrdfDirectory
        {
            get
            {
                Status["robotNameReceived"].WaitOne();
                return Path.Combine(localUrdfDirectory, RobotName);
            }
        }

        public UrdfTransferFromRos(RosSocket rosSocket, string localUrdfDirectory, string urdfParameter, string robotNameParameter)
        {
            RosSocket = rosSocket;
            this.localUrdfDirectory = localUrdfDirectory;
            this.urdfParameter = urdfParameter;
            this.robotNameParameter = robotNameParameter;


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
                                                                                    new rosapi.GetParamRequest(robotNameParameter, DEFAULT_STRING));

            var robotDescriptionReceiver = new ServiceReceiver<rosapi.GetParamRequest, rosapi.GetParamResponse>(RosSocket, "/rosapi/get_param",
                                                                                        new rosapi.GetParamRequest(urdfParameter, DEFAULT_STRING),
                                                                                        Path.DirectorySeparatorChar + CutAfterColon(urdfParameter) + ".urdf");

            robotDescriptionReceiver.ReceiveEventHandler += ReceiveRobotDescription;
        }
        private void ReceiveRobotName(object serviceResponse)
        {
            RobotName = FormatTextFileContents(((rosapi.GetParamResponse)serviceResponse).value);
            Status["robotNameReceived"].Set();
        }

        private void ReceiveRobotDescription(ServiceReceiver<rosapi.GetParamRequest, rosapi.GetParamResponse> serviceReceiver, rosapi.GetParamResponse serviceResponse)
        {
            string robotDescription = FormatTextFileContents(serviceResponse.value);

            Thread importResourceFilesThread = new Thread(() => ImportResourceFiles(robotDescription));
            importResourceFilesThread.Start();

            Thread writeTextFileThread = new Thread(() => WriteTextFile((string)serviceReceiver.HandlerParameter, robotDescription));
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
            Uri resourceFileUri = new Uri(serviceReceiver.ServiceParameter.name);

            if (IsColladaFile(resourceFileUri))
            {
                Thread importResourceFilesThread = new Thread(() => ImportColladaTextureFiles(resourceFileUri, System.Text.Encoding.UTF8.GetString(fileContents)));
                importResourceFilesThread.Start();
            }
            else
                UpdateFileRequestStatus(resourceFileUri.ToString());

            Thread writeTextFileThread = new Thread(() => WriteBinaryResponseToFile((string)serviceReceiver.HandlerParameter, fileContents));
            writeTextFileThread.Start();
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

            //Only update DAE file request status once its texture files have been requested
            UpdateFileRequestStatus(daeFileUri.ToString());
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

        private static string CutAfterColon(string input)
        {
            int colonIndex = input.IndexOf(':');
            if (colonIndex >= 0)
            {
                input = input.Substring(colonIndex + 1);
            }
            return input;
        }

        private static string FormatTextFileContents(string fileContents)
        {
            // remove enclosing quotations if existent:
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
