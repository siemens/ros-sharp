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

* Validation to prevent files from being written outside the local URDF directory.
* Robot name sanitization and fallback extraction from the URDF.
* Import resource files sequentially.
* Configurable retry attempts, retry delays, and service timeouts.
* Cancellation support for resource file downloads.
* Progress tracking for resource file imports.
* Added recursive texture discovery for Collada files.
* Logging for transfer progress, failures, cancellation, and parsing errors.
* Explicit success and failure states for resource file imports.
* Save URDF files using the sanitized robot name.
    (C) Siemens AG, 2026, Mehmet Emre Cakal (emre.cakal@siemens.com/m.emrecakal@gmail.com)
*/

using System;
using System.IO;
using System.Collections.Generic;
using System.Text.RegularExpressions;
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
        private string robotDescription;

        public string LocalUrdfDirectory
        {
            get
            {
                Status["robotNameReceived"].WaitOne();
                return Path.Combine(localUrdfDirectory, RobotName);
            }
        }

        public static int MaxRetries { get; set; } = 3;
        public static int RetryDelayMs { get; set; } = 200;
        public static int ServiceTimeoutMs { get; set; } = 8000;
        public float ResourceFileImportProgress { get; private set; } = 0f;

        private readonly Queue<Uri> _pendingFiles = new Queue<Uri>();
        private readonly HashSet<string> _seenFiles = new HashSet<string>();
        private readonly CancellationToken cancellationToken;

        public UrdfTransferFromRos(
            RosSocket rosSocket,
            string localUrdfDirectory,
            string urdfParameter,
            string robotNameParameter,
            Log log,
            CancellationToken cancellationToken = default,
            int maxRetries = 3,
            int retryDelayMs = 200,
            int serviceTimeoutMs = 8000)
        {
            RosSocket = rosSocket;
            this.localUrdfDirectory = localUrdfDirectory;
            this.urdfParameter = urdfParameter;
            this.robotNameParameter = robotNameParameter;
            this.log = log;
            this.cancellationToken = cancellationToken;
            MaxRetries = maxRetries;
            RetryDelayMs = retryDelayMs;
            ServiceTimeoutMs = serviceTimeoutMs;

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
            log("Requesting robot description from ROS parameter: " + urdfParameter);

            var robotDescriptionReceiver = new ServiceReceiver<rosapi.GetParamRequest, rosapi.GetParamResponse>(
                RosSocket, "/rosapi/get_param",
                new rosapi.GetParamRequest(urdfParameter, DEFAULT_STRING),
                null);

            robotDescriptionReceiver.ReceiveEventHandler += ReceiveRobotDescription;

            RosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>(
                "/rosapi/get_param",
                ReceiveRobotName,
                new rosapi.GetParamRequest(robotNameParameter, CutAfterColon(robotNameParameter)));
        }

        private void ReceiveRobotName(object serviceResponse)
        {
            // try to extract robot name from the name param response
            string robotNameFromResponse = ((rosapi.GetParamResponse)serviceResponse).value.Trim('"');
            if (!string.IsNullOrEmpty(robotNameFromResponse))
            {
                RobotName = SanitizeRobotName(NormalizeRosString(robotNameFromResponse));
                Status["robotNameReceived"].Set();
                return;
            }

            // if not, wait for robot description (guaranteed to arrive first) and extract name from URDF
            Status["robotDescriptionReceived"].WaitOne();
            string robotNameFromUrdf = GetRobotNameFromRobotDescription(robotDescription);

            if (!string.IsNullOrEmpty(robotNameFromUrdf))
            {
                log($"Received invalid robot name response (\"{robotNameFromResponse}\"). Extracted robot name from URDF: {robotNameFromUrdf}");
                RobotName = SanitizeRobotName(robotNameFromUrdf);
            }
            // if the robot name cannot be extracted from URDF, use a default name
            else
            {
                log($"Received invalid robot name response (\"{robotNameFromResponse}\"), and could not extract robot name from URDF. Using node name: \"unnamed_robot\" as robot name.");
                RobotName = "unnamed_robot";
            }

            Status["robotNameReceived"].Set();
        }


        private void ReceiveRobotDescription(
            ServiceReceiver<rosapi.GetParamRequest, rosapi.GetParamResponse> serviceReceiver,
            rosapi.GetParamResponse serviceResponse)
        {
            robotDescription = NormalizeRosString(serviceResponse.value);
            Status["robotDescriptionReceived"].Set();

            Thread importResourceFilesThread = new Thread(() => ImportResourceFilesSequentially(robotDescription));
            importResourceFilesThread.Start();

            Thread writeUrdfFileThread = new Thread(() => WriteUrdfFile(robotDescription));
            writeUrdfFileThread.Start();
        }

        // Fetch resource files one at a time, with per file retry logic.
        // Collada (.dae) files are parsed on arrival and their textures are appended
        private void ImportResourceFilesSequentially(string urdfContents)
        {
            try
            {
                List<Uri> initialUris = ReadResourceFileUris(XDocument.Parse(urdfContents));
                foreach (Uri uri in initialUris)
                    EnqueueIfNew(uri);

                if (_pendingFiles.Count == 0)
                {
                    ResourceFileImportProgress = 1f;
                    log("No resource files to import.");
                    return;
                }

                int processed = 0;
                int totalFiles = _pendingFiles.Count;
                ResourceFileImportProgress = 0f;
                while (_pendingFiles.Count > 0)
                {
                    if (cancellationToken.IsCancellationRequested)
                    {
                        log("Resource file import cancelled.");
                        ResourceFileImportProgress = -1f;
                        return;
                    }

                    Uri fileUri = _pendingFiles.Dequeue();
                    processed++;
                    ResourceFileImportProgress = (float)processed / totalFiles;
                    log($"Fetching resource file ({processed}, {_pendingFiles.Count} remaining in queue): {fileUri}, progress: {ResourceFileImportProgress:P1}");

                    byte[] fileContents = RequestFileWithRetry(fileUri);
                    if (fileContents == null)
                    {
                        string reason = cancellationToken.IsCancellationRequested
                            ? "Cancelled"
                            : $"Failed after {MaxRetries} attempts";

                        log($"{fileUri}: {reason}. Aborting resource file import.");
                        ResourceFileImportProgress = -1f;
                        return;
                    }

                    // Write the file to disk
                    WriteBinaryResponseToFile(GetLocalFilename(fileUri), fileContents);

                    // If this is a Collada file, discover and enqueue its textures before moving on
                    if (IsColladaFile(fileUri))
                    {
                        try
                        {
                            XDocument xDoc = XDocument.Parse(System.Text.Encoding.UTF8.GetString(fileContents));
                            foreach (Uri textureUri in ReadDaeTextureUris(fileUri, xDoc))
                            {
                                if (EnqueueIfNew(textureUri))
                                    totalFiles++;
                            }
                        }
                        catch (Exception e)
                        {
                            log($"Could not parse Collada file for textures ({fileUri}): {e.Message}. Aborting resource file import.");
                            ResourceFileImportProgress = -1f;
                            return;
                        }
                    }

                    // Keep FilesBeingProcessed populated so external callers can read .Count
                    lock (FilesBeingProcessed)
                        FilesBeingProcessed[fileUri.ToString()] = true;
                }

                ResourceFileImportProgress = 1f;
            }
            catch (Exception e)
            {
                ResourceFileImportProgress = -1f;
                log($"Resource file import failed: {e.Message}");
            }
            finally
            {
                Status["resourceFilesReceived"].Set();
            }
        }

        private bool EnqueueIfNew(Uri uri)
        {
            if (!_seenFiles.Add(uri.ToString()))
                return false;

            _pendingFiles.Enqueue(uri);
            return true;
        }

        // Calls the file_server service for a single URI. Retries up to MaxRetries times.
        // Returns null if the file could not be fetched or if the cancellation token was triggered.
        private byte[] RequestFileWithRetry(Uri fileUri)
        {
            for (int attempt = 1; attempt <= MaxRetries; attempt++)
            {
                // 1. Check for cancellation before making the service call
                if (cancellationToken.IsCancellationRequested)
                    return null;

                byte[] result = null;
                var responseEvent = new ManualResetEvent(false);

                RosSocket.CallService<file_server.GetBinaryFileRequest, file_server.GetBinaryFileResponse>(
                    "/file_server/get_file",
                    (file_server.GetBinaryFileResponse response) =>
                    {
                        result = response.value;
                        responseEvent.Set();
                    },
                    new file_server.GetBinaryFileRequest(fileUri.ToString()));

                // 2. Wait for either the service response or cancellation
                int signaledIndex = WaitHandle.WaitAny(
                    new WaitHandle[] { responseEvent, cancellationToken.WaitHandle },
                    ServiceTimeoutMs);

                if (signaledIndex == 0)
                    return result;

                if (signaledIndex == 1)
                    return null;

                log($"Attempt {attempt}/{MaxRetries} timed out for {fileUri}.");

                // 3. Check for cancellation before retrying
                if (attempt < MaxRetries && cancellationToken.WaitHandle.WaitOne(RetryDelayMs))
                {
                    return null;
                }
            }
            return null;
        }

        private void WriteBinaryResponseToFile(string relativeLocalFilename, byte[] fileContents)
        {
            // CWE22: Ensure that the file is written to a path within the allowed local URDF directory
            string basePath = Path.GetFullPath(LocalUrdfDirectory);
            string filename = Path.GetFullPath(Path.Combine(basePath, relativeLocalFilename.TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)));
            if (!filename.StartsWith(basePath + Path.DirectorySeparatorChar, StringComparison.Ordinal))
                throw new UnauthorizedAccessException("Path escapes the allowed directory: " + filename);

            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            File.WriteAllBytes(filename, fileContents);
        }

        private void WriteUrdfFile(string fileContents)
        {
            try {
                // CWE22: Ensure that the file is written to a path within the allowed local URDF directory
                string basePath = Path.GetFullPath(LocalUrdfDirectory);
                string filename = Path.GetFullPath(Path.Combine(basePath, RobotName + ".urdf"));
                if (!filename.StartsWith(basePath + Path.DirectorySeparatorChar, StringComparison.Ordinal))
                    throw new UnauthorizedAccessException("Path escapes the allowed directory: " + filename);

                Directory.CreateDirectory(Path.GetDirectoryName(filename));
                File.WriteAllText(filename, fileContents);
            }
            catch (Exception e)
            {
                log("Could not write urdf file: " + e.Message);
            }
        }

        private static string GetLocalFilename(Uri resourceFilePath)
        {
            return Path.DirectorySeparatorChar
                + resourceFilePath.Host
                + resourceFilePath.LocalPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
        }

        private static string SanitizeRobotName(string robotName)
        {
            string sanitized = Regex.Replace(robotName ?? string.Empty, @"[^a-zA-Z0-9_\-]", "_");
            return string.IsNullOrEmpty(sanitized) || sanitized.Trim('_') == string.Empty ? "unnamed_robot" : sanitized;
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

        private static string NormalizeRosString(string fileContents)
        {
            // remove enclosing quotations if existent:
            if (fileContents.Substring(0, 1) == "\"" && fileContents.Substring(fileContents.Length - 1, 1) == "\"")
                fileContents = fileContents.Substring(1, fileContents.Length - 2);

            // replace \" quotation sign by actual quotation:
            fileContents = fileContents.Replace("\\\"", "\"");

            // replace \n newline sign by actual new line:
            return fileContents.Replace("\\n", Environment.NewLine);
        }

        private string GetRobotNameFromRobotDescription(string robotDescription)
        {
            try
            {
                var xDocument = XDocument.Parse(robotDescription);
                var root = xDocument.Root;
                if (root != null && root.Name.LocalName.Equals("robot", StringComparison.OrdinalIgnoreCase))
                    return root.Attribute("name")?.Value;

                var robotElem = xDocument.Descendants().FirstOrDefault(e => e.Name.LocalName.Equals("robot", StringComparison.OrdinalIgnoreCase));
                return robotElem?.Attribute("name")?.Value;
            }
            catch (Exception e)
            {
                log("Could not parse robot description to get robot name: " + e.Message);
                return null;
            }
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
