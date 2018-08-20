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
        private static string assetRootFolder;
        private static string meshRootFolder;

        #region SetAssetRootFolder
        public static void SetAssetRootFolder(Robot robot)
        {
            assetRootFolder = GetRelativePathToParentDirectory(robot.filename);
        }

        public static void SetAssetRootFolder(string newPath)
        {
            assetRootFolder = newPath;
        }
        #endregion

        public static void SetMeshRootFolder(string newPath)
        {
            meshRootFolder = newPath;
        }

        #region GetPaths
        public static string GetRelativePathToParentDirectory(this string urdfFile)
        {
            var directoryAbsolutePath = Path.GetDirectoryName(urdfFile);
            return GetRelativeAssetPath(directoryAbsolutePath);
        }

        public static string GetAssetRootFolder()
        {
            return assetRootFolder;
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

        #region GetMeshPaths
        public static string GetNewMeshPath(string meshFileName)
        {
            //meshFileName is the name of the mesh, with file extension
            return Path.Combine(meshRootFolder, meshFileName);
        }

        public static string GetPackagePathForMesh(string absoluteMeshPath)
        {
            string relativeMeshPath = meshRootFolder.Substring(assetRootFolder.Length + 1);
            return Path.Combine("package://", relativeMeshPath, Path.GetFileName(absoluteMeshPath)).Replace("\\", "/");
        }
        #endregion

        public static bool IsValidAssetPath(string path)
        {
            return GetRelativeAssetPath(path) != null;
        }
    }

}