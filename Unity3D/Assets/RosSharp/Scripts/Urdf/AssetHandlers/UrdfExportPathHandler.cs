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
using System.IO;
using UnityEngine;

namespace RosSharp.Urdf
{
    public static class UrdfExportPathHandler
    {
        //absolute path to export folder
        private static string exportRoot;
        //Relative to export root folder
        private static string exportDestination;

        private const string MeshFolderName = "meshes";
        private const string ResourceFolderName = "resources";

        public static void SetExportRoot(string newPath)
        {
            exportRoot = newPath;
        }

        public static bool SetExportDestination(string newDestination)
        {
            if (newDestination.Contains(exportRoot))
            {
                exportDestination = (newDestination == exportRoot ? "" : newDestination.Substring(exportRoot.Length + 1));

                Directory.CreateDirectory(Path.Combine(GetExportDestination(), MeshFolderName));
                Directory.CreateDirectory(Path.Combine(GetExportDestination(), ResourceFolderName));
                return true;
            }

            Debug.LogWarning("Export destination must be the same as the root folder or a subfolder of it.");
            exportDestination = "";
            return false;
        }

        public static string GetExportDestination()
        {
            return exportDestination == null ? exportRoot : Path.Combine(exportRoot, exportDestination);
        }

        #region GetExportPaths
        //Returns an absolute path to the export destination for the mesh
        //meshFileName includes the file extension
        public static string GetNewMeshPath(string meshFileName)
        {
            return Path.Combine(exportRoot, exportDestination, MeshFolderName, meshFileName);
        }

        //Returns an absolute path to the new resource
        public static string GetNewResourcePath(string resourceFileName)
        {
            return Path.Combine(exportRoot, exportDestination, ResourceFolderName, resourceFileName);
        }

        public static string GetPackagePathForMesh(string meshPath)
        {
            return Path.Combine("package://", exportDestination, MeshFolderName, Path.GetFileName(meshPath)).Replace("\\", "/");
        }

        public static string GetPackagePathForResource(string resourcePath)
        {
            return Path.Combine("package://", exportDestination, ResourceFolderName, Path.GetFileName(resourcePath)).Replace("\\", "/");
        }
        #endregion

        public static void Clear()
        {
            exportRoot = "";
            exportDestination = "";
        }
    }

}