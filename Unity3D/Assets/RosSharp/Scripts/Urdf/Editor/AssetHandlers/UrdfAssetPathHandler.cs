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
            var absolutePathUnityFormat = absolutePath.SetSeparatorChar();
            if (!absolutePathUnityFormat.StartsWith(Application.dataPath.SetSeparatorChar()))
                return null;

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
            if (!urdfPath.StartsWith(@"package://"))
            {
                Debug.LogWarning(urdfPath + " is not a valid URDF package file path. Path should start with \"package://\".");
                return null;
            }

            var path = urdfPath.Substring(10).SetSeparatorChar();

            if (Path.GetExtension(path)?.ToLowerInvariant() == ".stl")
                path = path.Substring(0, path.Length - 3) + "prefab";

            return Path.Combine(packageRoot, path);
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