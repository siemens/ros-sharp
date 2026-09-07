/*
© Siemens AG, 2018-2019
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

* Async methods are used to handle the transfer of URDF files to ROS.
* Added USINGWITHUNITY directive to ensure that the log messages are displayed according to the environment.
    * Comment out the USINGWITHUNITY directive to display log messages in the .NET console.
* Instead of relying on the service response, file count system is used to determine the completion of the transfer.
* Removed additional RosConnector instance. Urdf transfer is now handled by the existing RosConnector component.
* If no RosConnector component is present in the scene, one can be created by pressing the button.
* RosConnector specific input fields have been removed as they are no longer required.
* Robot name parameter input field added.
    (C) Siemens AG, 2024, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)

* Add validation and logging for empty robot-name service responses.
* Add logging for robot-description publication responses.
* Add validation for empty file-server responses.
* Add progress tracking for exported resource files.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/


using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Threading;
using System.Xml.Linq;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;
using file_server = RosSharp.RosBridgeClient.MessageTypes.FileServer;
using System.Text.Json;


namespace RosSharp.RosBridgeClient.UrdfTransfer
{
    public class UrdfTransferToRos : UrdfTransfer
    {
        private string urdfFilePath;
        private string assetRootFolder;
        private string rosPackage;
        private string robotNameParameter;
        private int numUris;
        private int sentFileCountSoFar = 0;
        private int totalValidFileCount;

        public float ResourceFileExportProgress { get; private set; } = 0f;

        public UrdfTransferToRos(
            RosSocket rosSocket,
            string robotName,
            string robotNameParameter,
            string urdfFilePath,
            string rosPackage,
            Log log)
        {
            RosSocket = rosSocket;
            RobotName = robotName;
            this.urdfFilePath = urdfFilePath;
            this.rosPackage = rosPackage;
            this.log = log;
            this.robotNameParameter = robotNameParameter;

            Status = new Dictionary<string, ManualResetEvent>
            {
                {"robotNamePublished", new ManualResetEvent(false)},
                {"robotDescriptionPublished", new ManualResetEvent(false)},
                {"resourceFilesSent", new ManualResetEvent(false)}
            };

            FilesBeingProcessed = new Dictionary<string, bool>();
        }

        public override async void Transfer()
        {
            // Publish robot name
            await PublishRobotName();
            await PublishRobotDescription();
            await PublishResourceFiles();
        }

        private async Task PublishRobotName()
        {
            await Task.Run(() => RosSocket.CallService<rosapi.SetParamRequest, rosapi.SetParamResponse>(
                "/rosapi/set_param",
                SetRobotNameHandler,
                new rosapi.SetParamRequest(
                    JsonSerializer.Serialize(robotNameParameter),
                    JsonSerializer.Serialize(RobotName)
                )));
        }

        private async Task PublishRobotDescription()
        {
            XDocument urdfXDoc = XDocument.Load(urdfFilePath);
            FixPackagePaths(urdfXDoc);

            // Publish /robot_description param
            await Task.Run(() => RosSocket.CallService<rosapi.SetParamRequest, rosapi.SetParamResponse>(
                "/rosapi/set_param",
                SetRobotDescriptionHandler,
                new rosapi.SetParamRequest(
                    JsonSerializer.Serialize(rosPackage),
                    JsonSerializer.Serialize(urdfXDoc.ToString())
                )));

            // Send URDF file to ROS package
            string urdfPackagePath = "package://" + CutBeforeColon(rosPackage) + "/" + Path.GetFileName(urdfFilePath);
            string urdfFileContents = "<?xml version='1.0' encoding='utf-8'?>\n" + urdfXDoc.ToString();
            byte[] urdfBytes = System.Text.Encoding.UTF8.GetBytes(urdfFileContents);

            await SendFileToRos(urdfPackagePath, urdfBytes);
        }

        private async Task PublishResourceFiles()
        {
            XDocument xDocument = XDocument.Load(urdfFilePath);
            numUris = xDocument.Descendants().Count(HasValidResourcePath);
            List<Uri> resourceFileUris = ReadResourceFileUris(xDocument);
            totalValidFileCount = resourceFileUris.Count;

            bool badUriInFile = numUris != totalValidFileCount;
            if (totalValidFileCount == 0 && !badUriInFile)
            {
                Status["resourceFilesSent"].Set();
                Console.WriteLine("No resource files to send.");
            }
            else
            {
                if (badUriInFile)
                    FilesBeingProcessed["UnreadableUri"] = false;

                assetRootFolder = FindAssetRootFolder(resourceFileUris[0].ToString());
                await PublishFiles(resourceFileUris);
            }
        }

        private async Task PublishFiles(List<Uri> resourceFileUris)
        {
            foreach (Uri resourceFilePath in resourceFileUris)
            {
                string newPackagePath = GetNewPackagePath(resourceFilePath);
                if (FilesBeingProcessed.ContainsKey(newPackagePath)) continue;

                string absolutePath = Path.Combine(assetRootFolder, resourceFilePath.ToString().Substring("package://".Length));
                if (IsColladaFile(resourceFilePath))
                {
                    List<Uri> colladaTextureFiles = ReadDaeTextureUris(resourceFilePath, XDocument.Load(absolutePath));
                    totalValidFileCount += colladaTextureFiles.Count;
                    await PublishFiles(colladaTextureFiles);
                }

                try
                {
                    byte[] fileContents = File.ReadAllBytes(absolutePath);
                    await SendFileToRos(newPackagePath, fileContents);
                }
                catch (IOException)
                {
                    Console.WriteLine("Transfer to ROS: Could not find file " + absolutePath + ".");
                }
            }
        }

        private async Task SendFileToRos(string rosPackagePath, byte[] fileContents)
        {
            var taskCompletionSource = new TaskCompletionSource<bool>();

            RosSocket.CallService<file_server.SaveBinaryFileRequest, file_server.SaveBinaryFileResponse>(
                "/file_server/save_file",
                response =>
                {
                    if (!string.IsNullOrEmpty(response.name))
                    {
                        taskCompletionSource.SetResult(true);
                    }
                    else
                    {
                        log("ERROR: RosSocket CallService /file_server/save_file response is null.");
                        // TODO: should terminate, all or nothing, or retry?
                        taskCompletionSource.SetResult(false);
                    }

                },
                new file_server.SaveBinaryFileRequest(rosPackagePath, fileContents));


            if (!FilesBeingProcessed.ContainsKey(rosPackagePath))
            {
                FilesBeingProcessed.Add(rosPackagePath, false);
            }
            else
            {
                Console.WriteLine("File path already exists in FilesBeingProcessed");
            }

            var response = await taskCompletionSource.Task;
            Console.WriteLine("File sent to ROS: " + rosPackagePath);
            SaveFileResponseHandler();

        }

        // Finds which part of the package path is common to urdfFilePath.
        // Based on that, determines the root folder that contains all URDF files.
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

        // If the package paths defined in the URDF do not contain the package defined by the user in the
        // URDF Publisher window, then add rosPackage to the package path. 
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
                        Console.WriteLine("Filename " + xElement.Attribute("filename").Value +
                                          " is not formatted correctly.\n" + e);
                    }
            }
        }

        private string GetNewPackagePath(Uri originalPath)
        {
            string packageName = originalPath.Host;
            if (packageName.Equals(CutBeforeColon(rosPackage)))
                return originalPath.ToString();

            return "package://" + CutBeforeColon(rosPackage) + "/" + packageName + originalPath.AbsolutePath;
        }

        private void SetRobotNameHandler(rosapi.SetParamResponse serviceResponse)
        {
            if (string.IsNullOrEmpty(JsonSerializer.Serialize(serviceResponse)))
            {
                log("Service response robot name set_param is null.");
            }
            else
            {
                log("Service response robot name set_param: " + JsonSerializer.Serialize(serviceResponse));
            }
            Status["robotNamePublished"].Set();
        }

        private void SetRobotDescriptionHandler(rosapi.SetParamResponse serviceResponse)
        {
            log("Service response robot description set_param: " + JsonSerializer.Serialize(serviceResponse));
            Status["robotDescriptionPublished"].Set();
        }

        private void SaveFileResponseHandler()
        {
            sentFileCountSoFar++;
            ResourceFileExportProgress = (float)sentFileCountSoFar / (float)(totalValidFileCount + 1); // +1 for the URDF file itself

            log("Sent: " + sentFileCountSoFar + " of " + (totalValidFileCount + 1) + " files. Progress: " + ResourceFileExportProgress.ToString("P2"));

            if (numUris != 0 && sentFileCountSoFar == totalValidFileCount + 1)
            {
                if (Status["robotNamePublished"].Equals(false) || Status["robotDescriptionPublished"].Equals(false))
                {
                    log("Error: Robot name and description should be published before resource files.");
                }
                Status["resourceFilesSent"].Set();
                Console.WriteLine("All resource files sent. Closing connection.");
                RosSocket.Close();
            }
        }

        private static string CutBeforeColon(string input)
        {
            int colonIndex = input.IndexOf(':');
            if (colonIndex >= 0)
            {
                input = input.Substring(0, colonIndex);
            }
            return input;
        }

    }
}
