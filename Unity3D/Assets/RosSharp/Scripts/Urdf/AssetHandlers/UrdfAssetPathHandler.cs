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
using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    public static class UrdfAssetPathHandler
    {
        //Relative to Assets folder
        private static string packageRoot;
        //Relative to package root folder
        private static string exportDestination;

        private static string meshFolderName = "meshes";
        private static string resourceFolderName = "resources";

        #region SetAssetRootFolder
        public static void SetPackageRoot(string newPath, bool correctingIncorrectPackageRoot = false)
        {
            string oldPackagePath = packageRoot;

            packageRoot = GetRelativeAssetPath(newPath);

            if(correctingIncorrectPackageRoot)
                UrdfMaterialHandler.MoveMaterialsToNewLocation(oldPackagePath);
        }
        #endregion

        public static void SetExportDestination(string newPath)
        {
            string relativeExportPath = GetRelativeAssetPath(newPath);
            exportDestination = relativeExportPath == packageRoot ? "" : relativeExportPath.Substring(packageRoot.Length+1);

            //Create resource folders if they don't already exist
            if (!AssetDatabase.IsValidFolder(Path.Combine(GetExportDestination(), meshFolderName)))
                AssetDatabase.CreateFolder(GetExportDestination(), meshFolderName);
            if (!AssetDatabase.IsValidFolder(Path.Combine(GetExportDestination(), resourceFolderName)))
                AssetDatabase.CreateFolder(GetExportDestination(), resourceFolderName);
        }

        #region GetPaths
        public static string GetPackageRoot()
        {
            return packageRoot;
        }

        public static string GetExportDestination()
        {
            return exportDestination == null ? packageRoot : Path.Combine(packageRoot, exportDestination);
        }

        public static string GetRelativeAssetPath(string absolutePath)
        {
            var absolutePathUnityFormat = absolutePath.Replace(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            if (!absolutePathUnityFormat.StartsWith(Application.dataPath))
                return null;

            var assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
            return assetPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
        }

        public static string GetFullAssetPath(string relativePath)
        {
            return Application.dataPath + relativePath.Substring("Assets".Length);
        }
        #endregion

        #region GetExportPaths
        //Returns new mesh path, relative to the Assets folder
        public static string GetNewMeshPath(string meshFileName)
        {
            //meshFileName is the name of the mesh, with file extension
            return Path.Combine(packageRoot, exportDestination, meshFolderName, meshFileName);
        }

        //Returns new resource path, relative to the Assets folder
        public static string GetNewResourcePath(string resourceFileName)
        {
            return Path.Combine(packageRoot, exportDestination, resourceFolderName, resourceFileName);
        }

        public static string GetPackagePathForMesh(string meshPath)
        {
            return Path.Combine("package://", exportDestination, meshFolderName, Path.GetFileName(meshPath)).Replace("\\", "/");
        }

        public static string GetPackagePathForResource(string resourcePath)
        {
            return Path.Combine("package://", exportDestination, resourceFolderName, Path.GetFileName(resourcePath)).Replace("\\", "/");
        }
        #endregion

        public static bool IsValidAssetPath(string path)
        {
            return GetRelativeAssetPath(path) != null;
        }

        public static void Clear()
        {
            packageRoot = "";
            exportDestination = "";
        }
    }

}