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

* Prevent paths outside the Unity Assets folder from being accepted.
* Add canonical path resolution before validating asset paths.
* Prevent prefix-confusion paths from bypassing the Assets folder validation.
* Add support for resolving file:// URDF paths with traversal protection.
* Add warnings when unsafe or invalid paths are rejected.
    © Siemens AG 2026, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/
using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfAssetPathHandler
    {
        //Relative to Assets folder
        private static string packageRoot;
        private const string MaterialFolderName = "Materials";

        #region SetAssetRootFolder
        public static void SetPackageRoot(string newPath, bool correctingIncorrectPackageRoot = false)
        {
            string oldPackagePath = packageRoot;

            packageRoot = GetRelativeAssetPath(newPath);

            if(!AssetDatabase.IsValidFolder(Path.Combine(packageRoot, MaterialFolderName)))
                AssetDatabase.CreateFolder(packageRoot, MaterialFolderName);

            if (correctingIncorrectPackageRoot)
                MoveMaterialsToNewLocation(oldPackagePath);
        }
        #endregion

        #region GetPaths
        public static string GetPackageRoot()
        {
            return packageRoot;
        }
        
        public static string GetRelativeAssetPath(string absolutePath)
        {
            // CWE-22: Use GetFullPath to resolve any traversal sequences (../../) before comparing,
            // and append a separator to the data path to prevent prefix-confusion attacks
            string normalizedAbsolutePath = Path.GetFullPath(absolutePath).SetSeparatorChar();
            string normalizedApplicationDataPath = Path.GetFullPath(Application.dataPath).SetSeparatorChar()
                                                + Path.DirectorySeparatorChar;

            if (!normalizedAbsolutePath.StartsWith(normalizedApplicationDataPath, System.StringComparison.Ordinal))
            {
                Debug.LogWarning("Path is outside the Assets folder and cannot be used: " + absolutePath);
                return null;
            }

            var assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
            return assetPath.SetSeparatorChar();
        }

        public static string GetFullAssetPath(string relativePath)
        {
            string fullPath = Application.dataPath + relativePath.Substring("Assets".Length);
            return fullPath.SetSeparatorChar();
        }

        public static string GetRelativeAssetPathFromUrdfPath(string urdfPath)
        {
            if (urdfPath.StartsWith(@"package://"))
            {
                var path = urdfPath.Substring(10).SetSeparatorChar();

                if (Path.GetExtension(path)?.ToLowerInvariant() == ".stl")
                    path = path.Substring(0, path.Length - 3) + "prefab";

                return Path.Combine(packageRoot, path);
            }

            if (urdfPath.StartsWith("file://"))
            {
                // strip "file://" and normalise separators
                // (/opt/ros/… -> Assets/urdf/<robot_name>/opt/ros/…)
                var path = urdfPath.Substring("file://".Length)
                                   .SetSeparatorChar()
                                   .TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);

                // CWE-22: Reject any path component that would escape the package root via
                // traversal sequences (../../), even after normalisation.
                var combined = Path.GetFullPath(Path.Combine(
                    Path.GetFullPath(packageRoot), path));
                var packageRootFull = Path.GetFullPath(packageRoot)
                                      + Path.DirectorySeparatorChar;

                if (!combined.StartsWith(packageRootFull, System.StringComparison.Ordinal))
                {
                    Debug.LogWarning("Blocked file:// URI that resolves outside the package root: " + urdfPath);
                    return null;
                }

                if (Path.GetExtension(path)?.ToLowerInvariant() == ".stl")
                    path = path.Substring(0, path.Length - 3) + "prefab";

                return Path.Combine(packageRoot, path);
            }

            Debug.LogWarning(urdfPath + " is not a valid URDF package file path. Path should start with \"package://\".");
            return null;
        }
        #endregion

        public static bool IsValidAssetPath(string path)
        {
            return GetRelativeAssetPath(path) != null;
        }

        #region Materials

        private static void MoveMaterialsToNewLocation(string oldPackageRoot)
        {
            if (AssetDatabase.IsValidFolder(Path.Combine(oldPackageRoot, MaterialFolderName)))
                AssetDatabase.MoveAsset(
                    Path.Combine(oldPackageRoot, MaterialFolderName),
                    Path.Combine(UrdfAssetPathHandler.GetPackageRoot(), MaterialFolderName));
            else
                AssetDatabase.CreateFolder(UrdfAssetPathHandler.GetPackageRoot(), MaterialFolderName);
        }

        public static string GetMaterialAssetPath(string materialName)
        {
            return Path.Combine(packageRoot, MaterialFolderName, Path.GetFileName(materialName) + ".mat");
        }

        #endregion
    }

}