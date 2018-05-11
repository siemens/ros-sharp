﻿/*
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

namespace RosSharp.RosBridgeClient
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
            rosSocket.CallService("/rosapi/get_param", typeof(ParamValueString), receiveRobotName, new ParamName("/robot/name"));
            ServiceReceiver robotDescriptionReceiver = new ServiceReceiver(rosSocket, "/rosapi/get_param", new ParamName("/robot_description"), Path.DirectorySeparatorChar + "robot_description.urdf", typeof(ParamValueString));
            robotDescriptionReceiver.ReceiveEventHandler += receiveRobotDescription;

            return (WaitHandle.WaitAll(Status.Values.ToArray(), maxTimeOut));
        }

        private void receiveRobotName(object serviceResponse)
        {
            robotName = formatTextFileContents(((ParamValueString)serviceResponse).value);
            Status["robotNameReceived"].Set();
        }

        private void receiveRobotDescription(ServiceReceiver serviceReciever, object serviceResponse)
        {
            string robotDescription = formatTextFileContents(((ParamValueString)serviceResponse).value);

            Thread importResourceFilesThread = new Thread(() => importResourceFiles(robotDescription));
            importResourceFilesThread.Start();

            Thread writeTextFileThread = new Thread(() => writeTextFile((string)serviceReciever.HandlerParameter, robotDescription));
            writeTextFileThread.Start();

            Status["robotDescriptionReceived"].Set();
        }

        private void importResourceFiles(string fileContents)
        {
            List<ServiceReceiver> serviceReceivers = requestResourceFiles(readResourceFileUris(fileContents));
            foreach (ServiceReceiver serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += receiveResourceFile;
        }

        private static List<Uri> readResourceFileUris(string robotDescription)
        {
            XElement root = XElement.Parse(robotDescription);
            return (from seg in root.Descendants("mesh") where seg.Attribute("filename") != null select new Uri(seg.Attribute("filename").Value)).ToList();
        }

        private List<ServiceReceiver> requestResourceFiles(List<Uri> resourceFileUris)
        {
            List<ServiceReceiver> serviceReceivers = new List<ServiceReceiver>();
            foreach (Uri resourceFilePath in resourceFileUris)
            {
                if (!RequestedResourceFiles.ContainsKey(resourceFilePath))
                {
                    RequestedResourceFiles.Add(resourceFilePath, false);
                    serviceReceivers.Add(new ServiceReceiver(rosSocket, "/file_server/get_file", new ParamName(resourceFilePath.ToString()), getLocalFilename(resourceFilePath), typeof(ParamValueByte)));
                }
            }
            return serviceReceivers;
        }

        private void receiveResourceFile(ServiceReceiver serviceReceiver, object serviceResponse)
        {
            byte[] fileContents = ((ParamValueByte)serviceResponse).value;
            Uri resourceFileUri = new Uri(((ParamName)serviceReceiver.ServiceParameter).name);

            if (isColladaFile(resourceFileUri))
            {
                Thread importResourceFilesThread = new Thread(() => importDaeTextureFiles(resourceFileUri, System.Text.Encoding.UTF8.GetString(fileContents)));
                importResourceFilesThread.Start();
            }
            Thread writeTextFileThread = new Thread(() => writeBinaryResponseToFile((string)serviceReceiver.HandlerParameter, fileContents));
            writeTextFileThread.Start();

            updateFileRequestStatus(resourceFileUri);
        }

        private void updateFileRequestStatus(Uri resourceFileUri)
        {
            RequestedResourceFiles[resourceFileUri] = true;
            if (RequestedResourceFiles.Values.All(x => x == true))
                Status["resourceFilesReceived"].Set();
        }

        private static bool isColladaFile(Uri uri)
        {
            return Path.GetExtension(uri.LocalPath) == ".dae";
        }
        private void importDaeTextureFiles(Uri daeFileUri, string fileContents)
        {
            List<ServiceReceiver> serviceReceivers = requestResourceFiles(readDaeTextureUris(daeFileUri, fileContents));
            foreach (ServiceReceiver serviceReceiver in serviceReceivers)
                serviceReceiver.ReceiveEventHandler += receiveTextureFiles;
        }

        private List<Uri> readDaeTextureUris(Uri resourceFileUri, string fileContents)
        {
            XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
            XElement root = XElement.Parse(fileContents);
            return (from x in root.Elements()
                    where x.Name.LocalName == "library_images"
                    select new Uri(resourceFileUri, x.Element(xmlns + "image").Element(xmlns + "init_from").Value)).ToList();
        }

        private void receiveTextureFiles(ServiceReceiver serviceReceiver, object serviceResponse)
        {
            writeBinaryResponseToFile((string)serviceReceiver.HandlerParameter, ((ParamValueByte)serviceResponse).value);
            updateFileRequestStatus(new Uri(((ParamName)serviceReceiver.ServiceParameter).name));
        }

        private void writeBinaryResponseToFile(string relativeLocalFilename, byte[] fileContents)
        {
            string filename = LocalDirectory + relativeLocalFilename;
            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllBytes(filename, fileContents);
        }

        private void writeTextFile(string relativeLocalFilename, string fileContents)
        {
            string filename = LocalDirectory + relativeLocalFilename;
            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllText(filename, fileContents);
        }

        private static string getLocalFilename(Uri resourceFilePath)
        {
            return Path.DirectorySeparatorChar
                + resourceFilePath.Host
                + resourceFilePath.LocalPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
        }

        private static string formatTextFileContents(string fileContents)
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
