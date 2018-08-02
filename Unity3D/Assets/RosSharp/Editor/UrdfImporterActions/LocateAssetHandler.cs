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
using System;
using UnityEngine;
using UnityEditor;

namespace RosSharp.UrdfImporter
{
    public static class LocateAssetHandler
    {
        private static string fileAssetPath;

        public static T FindUrdfAsset<T>(string assetFileName) where T : UnityEngine.Object
        {
            fileAssetPath = GetAssetPathFromUrdfPath(assetFileName);
            T assetObject = AssetDatabase.LoadAssetAtPath<T>(fileAssetPath);

            if (assetObject != null)
                return assetObject;

            int option = EditorUtility.DisplayDialogComplex("Urdf Importer: Asset Not Found",
                    "Current root folder: " + UrdfAssetPathHandler.GetAssetRootFolder() +
                    "\n\nExpected asset path: " + fileAssetPath,
                    "Locate Asset",
                    "Ignore Missing Asset",
                    "Locate Root Folder");

            switch (option)
            {
                case 0:
                    assetObject = LocateAssetFile<T>();
                    break;
                case 1: break;
                case 2:
                    assetObject = LocateRootAssetFolder<T>(assetFileName);
                    break;
                default: break;
            }

            if (assetObject != null)
                return assetObject;

            ChooseFailureOption(fileAssetPath);
            return null;
        }

        private static T LocateRootAssetFolder<T>(string assetFileName) where T : UnityEngine.Object
        {
            T assetObject;
            string newAssetPath = EditorUtility.OpenFolderPanel(
                 "Locate package root folder",
                 Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                 "");

            UrdfAssetPathHandler.SetAssetRootFolder(UrdfAssetPathHandler.GetRelativeAssetPath(newAssetPath));
            fileAssetPath = GetAssetPathFromUrdfPath(assetFileName);
            assetObject = (T)AssetDatabase.LoadAssetAtPath(fileAssetPath, typeof(T));
            return assetObject;
        }

        private static T LocateAssetFile<T>() where T : UnityEngine.Object
        {
            T assetObject;
            string newPath = EditorUtility.OpenFilePanel(
                 "Couldn't find asset at " + fileAssetPath + ". Select correct file.",
                 Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                 "");

            fileAssetPath = UrdfAssetPathHandler.GetRelativeAssetPath(newPath);
            assetObject = (T)AssetDatabase.LoadAssetAtPath(fileAssetPath, typeof(T));
            return assetObject;
        }

        private static void ChooseFailureOption(string assetFilePath)
        {
            if (!EditorUtility.DisplayDialog(
                                "Urdf Importer: Missing Asset",
                                "Missing asset " + Path.GetFileName(assetFilePath) + " was ignored or could not be found.\n\nContinue URDF Import?",
                                "Yes",
                                "No"))
            {
                throw new InterruptedUrdfImportException("User cancelled URDF import. Model may be incomplete.");
            }
        }

        private static string GetAssetPathFromUrdfPath(string packagePath)
        {
            string path = packagePath.Substring(10).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);

            if (path.Substring(path.Length - 3, 3).ToLowerInvariant() == "stl")
                path = path.Substring(0, path.Length - 3) + "prefab";

            return Path.Combine(UrdfAssetPathHandler.GetAssetRootFolder(), path);
        }
    }

    class InterruptedUrdfImportException : Exception
    {
        public InterruptedUrdfImportException(string message)
            : base(message)
        {
        }
    }
}
