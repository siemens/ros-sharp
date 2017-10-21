/*
© Siemens AG, 2017
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
using UnityEngine;

namespace RosBridgeClient
{
    public delegate void ReceiveEventHandler(ServiceReceiver sender, object ServiceResponse);

    public class ServiceReceiver
    {
        public string ServiceName { get; private set; }
        public object ServiceParameter { get; private set; }
        public object HandlerParameter { get; set; }
        public event ReceiveEventHandler ReceiveEventHandler;

        public ServiceReceiver(RosSocket rosSocket, string service, object parameter, object handlerParameter, Type responseType)
        {
            ServiceName = service;
            ServiceParameter = parameter;
            HandlerParameter = handlerParameter;
            rosSocket.CallService(ServiceName, responseType, receive, ServiceParameter);
        }
        private void receive(object ServiceResponse)
        {
            if (ReceiveEventHandler != null)
                ReceiveEventHandler.Invoke(this, ServiceResponse);
        }
    }

    public class UrdfImporter
    {
        private RosSocket rosSocket;
        private string localDirectory;
        public string robotName { get; private set; }

        public string LocalDirectory
        {
            get
            {
                Status["robotNameReceived"].WaitOne();
                return Path.Combine(localDirectory, robotName);
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
            rosSocket.CallService("/rosapi/get_param", typeof(ParamValueString), ReceiveRobotName, new ParamName("/robot/name"));
            ServiceReceiver robotDescriptionReceiver = new ServiceReceiver(rosSocket, "/rosapi/get_param", new ParamName("/robot_description"), "/robot_description.urdf", typeof(ParamValueString));
            robotDescriptionReceiver.ReceiveEventHandler += ReceiveRobotDescription;

            return (WaitHandle.WaitAll(Status.Values.ToArray(), maxTimeOut));
        }

        private void ReceiveRobotName(object serviceResponse)
        {
            robotName = FormatTextFileContents(((ParamValueString)serviceResponse).value);
            Status["robotNameReceived"].Set();
        }

        private void ReceiveRobotDescription(ServiceReceiver serviceReciever, object serviceResponse)
        {
            string robotDescription = FormatTextFileContents(((ParamValueString)serviceResponse).value);

            Thread importResourceFilesThread = new Thread(() => ImportResourceFiles(robotDescription));
            importResourceFilesThread.Start();

            Thread writeTextFileThread = new Thread(() => WriteTextFile((string)serviceReciever.HandlerParameter, robotDescription));
            writeTextFileThread.Start();

            Status["robotDescriptionReceived"].Set();
        }

        private void ImportResourceFiles(string fileContents)
        {
            List<ServiceReceiver> serviceReceivers = RequestResourceFiles(ReadResourceFileUris(fileContents));

            foreach (ServiceReceiver serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += ReceiveResourceFile;
        }

        private static List<Uri> ReadResourceFileUris(string robotDescription)
        {
            XElement root = XElement.Parse(robotDescription);
            return (from seg in root.Descendants("mesh") where seg.Attribute("filename") != null select new Uri(seg.Attribute("filename").Value)).ToList();
        }

        private List<ServiceReceiver> RequestResourceFiles(List<Uri> resourceFileUris)
        {
            List<ServiceReceiver> serviceReceivers = new List<ServiceReceiver>();
            foreach (Uri resourceFilePath in resourceFileUris)
            {
                if (!RequestedResourceFiles.ContainsKey(resourceFilePath))
                {
                    RequestedResourceFiles.Add(resourceFilePath, false);
                    serviceReceivers.Add(new ServiceReceiver(rosSocket, "/file_server/get_file", new ParamName(resourceFilePath.ToString()), GetLocalFilename(resourceFilePath), typeof(ParamValueByte)));
                }
            }
            return serviceReceivers;
        }

        private void ReceiveResourceFile(ServiceReceiver serviceReceiver, object serviceResponse)
        {
            string fileContents = System.Text.Encoding.UTF8.GetString(((ParamValueByte)serviceResponse).value);

            Uri resourceFileUri = new Uri(((ParamName)serviceReceiver.ServiceParameter).name);

            if (IsColladaFile(resourceFileUri))
            {
                Thread importResourceFilesThread = new Thread(() => ImportDaeTextureFiles(resourceFileUri, fileContents));
                importResourceFilesThread.Start();
            }
            Thread writeTextFileThread = new Thread(() => WriteTextFile((string)serviceReceiver.HandlerParameter, fileContents));
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

        private void ImportDaeTextureFiles(Uri daeFileUri, string fileContents)
        {
            List<ServiceReceiver> serviceReceivers = RequestResourceFiles(ReadDaeTextureUris(daeFileUri, fileContents));
            foreach (ServiceReceiver serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += ReceiveTextureFiles;
        }

        private List<Uri> ReadDaeTextureUris(Uri resourceFileUri, string fileContents)
        {
            XElement root = XElement.Parse(fileContents);
            return (from x in root.Elements() where x.Name.LocalName == "library_images" select new Uri(resourceFileUri, x.Value)).ToList();
        }

        private void ReceiveTextureFiles(ServiceReceiver serviceReceiver, object serviceResponse)
        {
            WriteBinaryResponseToFile((string)serviceReceiver.HandlerParameter, ((ParamValueByte)serviceResponse).value);
            UpdateFileRequestStatus(new Uri(((ParamName)serviceReceiver.ServiceParameter).name));
        }

        private void WriteBinaryResponseToFile(string relativeLocalFilename, byte[] fileContents)
        {
            string filename = LocalDirectory + relativeLocalFilename;
            System.IO.Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllBytes(filename, fileContents);
        }

        private void WriteTextFile(string relativeLocalFilename, string fileContents)
        {
            string filename = LocalDirectory + relativeLocalFilename;
            System.IO.Directory.CreateDirectory(Path.GetDirectoryName(filename));
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
