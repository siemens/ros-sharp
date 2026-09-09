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

* Add path traversal protection to prevent the export subfolder from escaping the configured export root.
    © Siemens AG 2026, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/
using System;
using System.IO;
using UnityEngine;

namespace RosSharp.Urdf
{
    public static class UrdfExportPathHandler
    {
        //absolute path to export folder
        private static string exportRoot;
        //Relative to export root folder
        private static string subfolder;

        private const string MeshFolderName = "meshes";
        private const string ResourceFolderName = "resources";

        public static void SetExportPath(string root, string subRoot = "")
        {
            // Prevent a traversal subRoot (e.g. "../../etc") from escaping the export root (CWE-22)
            string normalizedRoot = Path.GetFullPath(root);
            string destination = Path.GetFullPath(Path.Combine(root, subRoot ?? string.Empty));

            if (!destination.StartsWith(normalizedRoot + Path.DirectorySeparatorChar, StringComparison.Ordinal)
                && destination != normalizedRoot)
            {
                Debug.LogError("URDF export failed: the export subfolder escapes the export root directory.\n" +
                               "Root: " + normalizedRoot + "\n" +
                               "Resolved destination: " + destination);
                throw new ArgumentException("Export subfolder must be inside the export root directory.");
            }

            exportRoot = root;
            subfolder = subRoot;

            Debug.Log("URDF export destination set to: " + GetExportDestination());
            Directory.CreateDirectory(GetExportDestination());
            Directory.CreateDirectory(Path.Combine(GetExportDestination(), MeshFolderName));
            Directory.CreateDirectory(Path.Combine(GetExportDestination(), ResourceFolderName));
        }

        #region GetExportPaths
        public static string GetExportDestination()
        {
            return subfolder == null ? exportRoot : Path.Combine(exportRoot, subfolder).SetSeparatorChar();
        }
        
        //Returns an absolute path to the export destination for the mesh
        //meshFileName includes the file extension
        public static string GetNewMeshPath(string meshFileName)
        {
            return Path.Combine(exportRoot, subfolder, MeshFolderName, meshFileName).SetSeparatorChar();
        }

        //Returns an absolute path to the new resource
        public static string GetNewResourcePath(string resourceFileName)
        {
            return Path.Combine(exportRoot, subfolder, ResourceFolderName, resourceFileName)
                .SetSeparatorChar();
        }

        public static string GetPackagePathForMesh(string meshPath)
        {
            //All package paths should use forward slashes
            return Path.Combine("package://", subfolder, MeshFolderName, Path.GetFileName(meshPath)).Replace("\\", "/");
        }

        public static string GetPackagePathForResource(string resourcePath)
        {
            //All package paths should use forward slashes
            return Path.Combine("package://", subfolder, ResourceFolderName, Path.GetFileName(resourcePath)).Replace("\\", "/");
        }
        #endregion

        public static void Clear()
        {
            exportRoot = "";
            subfolder = "";
        }
    }

}