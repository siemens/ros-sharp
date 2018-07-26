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
    public static class MissingAssetHandler
    {
        public static GameObject FindMissingAsset(string meshFileName)
        {
            string meshAssetPath = UrdfAssetDatabase.GetAssetPathFromPackagePath(meshFileName);
            GameObject meshObject = null;

            int option = EditorUtility.DisplayDialogComplex("Asset Not Found",
                     "Asset was not found at path " + meshAssetPath + ". Do you wish to locate the correct asset?",
                     "Locate Asset",
                     "Ignore",
                     "Select Root Folder");

            switch (option)
            {
                case 0:
                    string newMeshFilePath = EditorUtility.OpenFilePanel(
                       "Couldn't find asset at " + meshAssetPath + ". Select correct file.",
                       Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                       "");
                    meshAssetPath = UrdfAssetDatabase.GetAssetPathFromAbsolutePath(newMeshFilePath);
                    meshObject = AssetDatabase.LoadAssetAtPath<GameObject>(meshAssetPath);
                    break;
                case 1:
                    break;
                case 2:
                    string newAssetPath = EditorUtility.OpenFolderPanel(
                       "Locate package root folder",
                       Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                       "");
                    UrdfAssetDatabase.UpdateAssetPath(UrdfAssetDatabase.GetAssetPathFromAbsolutePath(newAssetPath));
                    meshAssetPath = UrdfAssetDatabase.GetAssetPathFromPackagePath(meshFileName);
                    meshObject = AssetDatabase.LoadAssetAtPath<GameObject>(meshAssetPath);
                    break;
                default:
                    break;
            }

            if (meshObject != null)
                return meshObject;
            else
            {
                ChooseFailureOption(meshAssetPath);
                return null;
            }

        }

        private static void ChooseFailureOption(string meshAssetPath)
        {
            if (!EditorUtility.DisplayDialog(
                                "Unable to Locate Asset",
                                "Asset " + meshAssetPath + " could not be found.",
                                "Continue Import",
                                "Cancel Import"))
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
