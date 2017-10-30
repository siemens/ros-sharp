/*
© Siemens AG, 2017
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

using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RosSharp
{
    public static class UrdfAssetPackageExporter
    {
        [MenuItem("Assets/Generate RosSharp Asset Package", false, 22)]
        private static void export()
        {
            List<string> Assets = new List<string>();
            Assets.AddRange(GetAssetsAtPath("RosSharp"));
            
            string fileName = EditorUtility.SaveFilePanel(
                     "Save URDF Unity Asset Package",
                     GetDefaultDirectory(),
                     "RosSharp.unitypackage",
                     "unitypackage");

            if (fileName.Length == 0)
                return;

            AssetDatabase.ExportPackage(Assets.ToArray(), fileName, ExportPackageOptions.Default);
        }
        private static List<string> GetAssetsAtPath(string path)
        {
            string[] assetFiles = Directory.GetFiles(Path.Combine(Application.dataPath, path), "*", SearchOption.AllDirectories);
            string[] metaFiles = Directory.GetFiles(Path.Combine(Application.dataPath, path), "*.meta", SearchOption.AllDirectories);

            return (from assetFile in assetFiles.Except(metaFiles)
                    select ("Assets" + assetFile.Substring(Application.dataPath.Length))
                    .Replace(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)).ToList();
        }

        private static string GetDefaultDirectory()
        {
            return Path.Combine(Directory.GetParent(Directory.GetParent(Application.dataPath).FullName).FullName, "Release");
        }


    }
}