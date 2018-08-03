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
        public static T FindUrdfAsset<T>(string urdfFileName) where T : UnityEngine.Object
        {
            string fileAssetPath = GetAssetPathFromUrdfPath(urdfFileName);
            T assetObject = AssetDatabase.LoadAssetAtPath<T>(fileAssetPath);

            if (assetObject != null)
                return assetObject;

            //If asset was not found, let user choose whether to search for
            //or ignore the missing asset.
            int option = EditorUtility.DisplayDialogComplex("Urdf Importer: Asset Not Found",
                "Current root folder: " + UrdfAssetPathHandler.GetAssetRootFolder() +
                "\n\nExpected asset path: " + fileAssetPath,
                "Locate Asset",
                "Ignore Missing Asset",
                "Locate Root Folder");
            
            switch (option)
            {
                case 0:
                    fileAssetPath = LocateAssetFile<T>(fileAssetPath);
                    break;
                case 1: break;
                case 2:
                    fileAssetPath = LocateRootAssetFolder<T>(urdfFileName);
                    break;
            }

            assetObject = (T) AssetDatabase.LoadAssetAtPath(fileAssetPath, typeof(T));
            if (assetObject != null)
                return assetObject;

            ChooseFailureOption(urdfFileName);
            return null;
        }

        private static string LocateRootAssetFolder<T>(string urdfFileName) where T : UnityEngine.Object
        {
            string newAssetPath = EditorUtility.OpenFolderPanel(
                "Locate package root folder",
                Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                "");

            UrdfAssetPathHandler.SetAssetRootFolder(UrdfAssetPathHandler.GetRelativeAssetPath(newAssetPath));

            return GetAssetPathFromUrdfPath(urdfFileName);
        }

        private static string LocateAssetFile<T>(string fileAssetPath) where T : UnityEngine.Object
        {
            string fileExtension = Path.GetExtension(fileAssetPath)?.Replace(".", "");

            string newPath = EditorUtility.OpenFilePanel(
                "Couldn't find asset at " + fileAssetPath + ". Select correct file.",
                UrdfAssetPathHandler.GetAssetRootFolder(),
                fileExtension);

            return UrdfAssetPathHandler.GetRelativeAssetPath(newPath);
        }

        private static void ChooseFailureOption(string urdfFilePath)
        {
            if (!EditorUtility.DisplayDialog(
                "Urdf Importer: Missing Asset",
                "Missing asset " + Path.GetFileName(urdfFilePath) +
                " was ignored or could not be found.\n\nContinue URDF Import?",
                "Yes",
                "No"))
            {
                throw new InterruptedUrdfImportException("User cancelled URDF import. Model may be incomplete.");
            }
        }

        private static string GetAssetPathFromUrdfPath(string urdfPath)
        {
            if (!urdfPath.StartsWith(@"package://"))
            {
                throw new InvalidFileNameException(urdfPath + " is not a valid URDF package file path. Path must start with \"package://\".");
            }

            var path = urdfPath.Substring(10)
                .Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);

            if (Path.GetExtension(path).ToLowerInvariant() == ".stl")
                path = path.Substring(0, path.Length - 3) + "prefab";

            return Path.Combine(UrdfAssetPathHandler.GetAssetRootFolder(), path);
        }

        private class InterruptedUrdfImportException : Exception
        {
            public InterruptedUrdfImportException(string message) : base(message)
            {
            }
        }

        private class InvalidFileNameException : Exception
        {
            public InvalidFileNameException(string message) : base(message)
            {
            }
        }
    }
}
