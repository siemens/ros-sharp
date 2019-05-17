/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Xml.Linq;
using Newtonsoft.Json;

using rosapi = RosSharp.RosBridgeClient.Services.RosApi;
using file_server = RosSharp.RosBridgeClient.Services.FileServer;

namespace RosSharp.RosBridgeClient.UrdfTransfer
{
    public class UrdfTransferToRos : UrdfTransfer
    {
        private string urdfFilePath;
        private string assetRootFolder;
        private string rosPackage;

        public UrdfTransferToRos(RosSocket rosSocket, string robotName, string urdfFilePath, string rosPackage)
        {
            RosSocket = rosSocket;
            RobotName = robotName;
            this.urdfFilePath = urdfFilePath;
            this.rosPackage = rosPackage;

            Status = new Dictionary<string, ManualResetEvent>
            {
                {"robotNamePublished", new ManualResetEvent(false)},
                {"robotDescriptionPublished", new ManualResetEvent(false)},
                {"resourceFilesSent", new ManualResetEvent(false)}
            };

            FilesBeingProcessed = new Dictionary<string, bool>();
        }

        public override void Transfer()
        {
            //Publish robot name param
            RosSocket.CallService<rosapi.SetParamRequest, rosapi.SetParamResponse>("/rosapi/set_param",
                SetRobotNameHandler,
                new rosapi.SetParamRequest("/robot/name", JsonConvert.SerializeObject(RobotName)));
            
            PublishRobotDescription();

            PublishResourceFiles();
        }

        private void PublishRobotDescription()
        {
            XDocument urdfXDoc = XDocument.Load(urdfFilePath);
            FixPackagePaths(urdfXDoc);

            //Publish /robot_description param
            RosSocket.CallService<rosapi.SetParamRequest, rosapi.SetParamResponse>("/rosapi/set_param",
                SetRobotDescriptionHandler,
                new rosapi.SetParamRequest("/robot_description", JsonConvert.SerializeObject(urdfXDoc.ToString())));

            //Send URDF file to ROS package
            string urdfPackagePath = "package://" + rosPackage + "/" + Path.GetFileName(urdfFilePath);
            string urdfFileContents = @"<?xml version='1.0' encoding='utf-8'?>\n" + urdfXDoc.ToString();
            byte[] urdfBytes = System.Text.Encoding.UTF8.GetBytes(urdfFileContents);
            
            SendFileToRos(urdfPackagePath, urdfBytes);
        }

        private void PublishResourceFiles()
        {
            XDocument xDocument = XDocument.Load(urdfFilePath);
            int numUris = xDocument.Descendants().Count(HasValidResourcePath);
            List<Uri> resourceFileUris = ReadResourceFileUris(xDocument);

            bool badUriInFile = numUris != resourceFileUris.Count;
            if (resourceFileUris.Count == 0 && !badUriInFile)
                Status["resourceFilesSent"].Set();
            else
            {
                if(badUriInFile)
                    FilesBeingProcessed["UnreadableUri"] = false;

                assetRootFolder = FindAssetRootFolder(resourceFileUris[0].ToString());
                PublishFiles(resourceFileUris);
            }
        }

        //If the package paths defined in the URDF do not contain the package defined by the user in the
        //URDF Publisher window, then add rosPackage to the package path. 
        private void FixPackagePaths(XDocument xDoc)
        {
            foreach (XElement xElement in xDoc.Descendants())
            {
                if (HasValidResourcePath(xElement))
                    try
                    {
                        Uri originalPath = new Uri(xElement.Attribute("filename").Value);
                        xElement.SetAttributeValue("filename", GetNewPackagePath(originalPath));
                    }
                    catch (UriFormatException e)
                    {

                    }
            }
        }

        private string GetNewPackagePath(Uri originalPath)
        {
            string packageName = originalPath.Host;
            if (packageName.Equals(rosPackage))
                return originalPath.ToString();

            return "package://" + rosPackage + "/" + packageName + originalPath.AbsolutePath;
        }

        private void PublishFiles(List<Uri> resourceFileUris)
        {
            foreach (Uri resourceFilePath in resourceFileUris)
            {
                string newPackagePath = GetNewPackagePath(resourceFilePath);
                if (FilesBeingProcessed.ContainsKey(newPackagePath)) continue;

                string absolutePath = Path.Combine(assetRootFolder, resourceFilePath.ToString().Substring("package://".Length));
                if (IsColladaFile(resourceFilePath))
                {
                    List<Uri> colladaTextureFiles = ReadDaeTextureUris(resourceFilePath, XDocument.Load(absolutePath));
                    PublishFiles(colladaTextureFiles);
                }

                try
                {
                    byte[] fileContents = File.ReadAllBytes(absolutePath);
                    SendFileToRos(newPackagePath, fileContents);
                }
                catch (IOException)
                {

                }
            }
        }

        private void SendFileToRos(string rosPackagePath, byte[] fileContents)
        {
            RosSocket.CallService<file_server.SaveBinaryFileRequest, file_server.SaveBinaryFileResponse>(
                "/file_server/save_file",
                SaveFileResponseHandler,
                new file_server.SaveBinaryFileRequest(rosPackagePath, fileContents));

            FilesBeingProcessed.Add(rosPackagePath, false);
        }

        //Finds which part of the package path is common to urdfFilePath.
        //Based on that, determines the root folder that contains all URDF files.
        private string FindAssetRootFolder(string packagePath)
        {
            if (!packagePath.Contains("package://"))
                return null;

            string suffix = Path.GetFileName(packagePath);
            string prefix = Path.GetDirectoryName(packagePath);
            prefix = prefix.Substring("package://".Length - 1);
            string absoluteFolderPath = Path.GetDirectoryName(urdfFilePath);

            while (true)
            {
                if (absoluteFolderPath.Contains(prefix) || prefix == "")
                    return absoluteFolderPath.Substring(0, absoluteFolderPath.Length - prefix.Length);

                suffix = Path.Combine(Path.GetFileName(prefix), suffix);
                prefix = Path.GetDirectoryName(prefix);
            }
        }

        private void SetRobotNameHandler(rosapi.SetParamResponse serviceResponse)
        {
            Status["robotNamePublished"].Set();
        }

        private void SetRobotDescriptionHandler(rosapi.SetParamResponse serviceResponse)
        {
            Status["robotDescriptionPublished"].Set();
        }

        private void SaveFileResponseHandler(file_server.SaveBinaryFileResponse response)
        {
            FilesBeingProcessed[response.name] = true;
            if (FilesBeingProcessed.Values.All(x => x == true))
                Status["resourceFilesSent"].Set();
        }
    }
}
