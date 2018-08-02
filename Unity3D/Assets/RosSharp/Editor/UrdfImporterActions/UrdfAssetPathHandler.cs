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
using UnityEngine;

namespace RosSharp.UrdfImporter
{
    public static class UrdfAssetPathHandler
    {
        private static string assetRootFolder;

        #region SetAssetRootFolder
        public static void SetAssetRootFolder(Robot robot)
        {
            assetRootFolder = GetPathToParentDirectory(robot.filename);
        }

        public static void SetAssetRootFolder(string newPath)
        {
            assetRootFolder = newPath;
        }

        public static string GetPathToParentDirectory(this string urdfFile)
        {
            string directoryAbsolutePath = Path.GetDirectoryName(urdfFile);
            return GetRelativeAssetPath(directoryAbsolutePath);
        }
        #endregion

        #region GetAssets
        public static string GetAssetRootFolder()
        {
            return assetRootFolder;
        }

        public static string GetRelativeAssetPath(string absolutePath)
        {
            string absolutePathUnityFormat = absolutePath.Replace(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            if (absolutePathUnityFormat.StartsWith(Application.dataPath))
            {
                string assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
                return assetPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            }
            return null;
        }
        #endregion

        public static bool IsValidAssetPath(string path)
        {
            return GetPathToParentDirectory(path) == null;
        }
    }
}