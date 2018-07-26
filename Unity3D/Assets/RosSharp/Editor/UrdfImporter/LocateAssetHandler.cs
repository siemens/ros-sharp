/*
© Siemens AG, 2017
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
        public static bool missingAsset;
        private static string fileAssetPath;

        public static T FindUrdfAsset<T>(string assetFileName) where T : UnityEngine.Object
        {
            fileAssetPath = UrdfAssetDatabase.GetAssetPathFromPackagePath(assetFileName);
            T assetObject = AssetDatabase.LoadAssetAtPath<T>(fileAssetPath);

            if (assetObject == null) // Asset is missing
            {
                if (!missingAsset) // Found an asset missing for the first time
                {
                    int option = EditorUtility.DisplayDialogComplex("Urdf Importer: Asset Not Found",
                         "Current root folder: " + UrdfAssetDatabase.GetAssetRootFolder() +
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
                }
                else
                {
                    bool locateAsset = EditorUtility.DisplayDialog("Urdf Importer: Asset Not Found",
                         "Expected asset path: " + fileAssetPath,
                         "Locate Asset",
                         "Ignore Missing Asset");

                    if (locateAsset)
                        assetObject = LocateAssetFile<T>();
                }
                missingAsset = true;
            }

            if (assetObject != null)
                return assetObject;
            else
            {
                ChooseFailureOption(fileAssetPath);
                return null;
            }
        }

        private static T LocateRootAssetFolder<T>(string assetFileName) where T : UnityEngine.Object
        {
            T assetObject;
            string newAssetPath = EditorUtility.OpenFolderPanel(
                 "Locate package root folder",
                 Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                 "");

            UrdfAssetDatabase.UpdateAssetPath(UrdfAssetDatabase.GetAssetPathFromAbsolutePath(newAssetPath));
            fileAssetPath = UrdfAssetDatabase.GetAssetPathFromPackagePath(assetFileName);
            assetObject = (T)AssetDatabase.LoadAssetAtPath(fileAssetPath, typeof(T));
            return assetObject;
        }

        private static T LocateAssetFile<T>() where T : UnityEngine.Object
        {
            T assetObject;
            string newAssetFilePath = EditorUtility.OpenFilePanel(
                 "Couldn't find asset at " + fileAssetPath + ". Select correct file.",
                 Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                 "");

            fileAssetPath = UrdfAssetDatabase.GetAssetPathFromAbsolutePath(newAssetFilePath);
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
    }

    class InterruptedUrdfImportException : Exception
    {
        public InterruptedUrdfImportException(string message)
            : base(message)
        {
        }
    }
}
