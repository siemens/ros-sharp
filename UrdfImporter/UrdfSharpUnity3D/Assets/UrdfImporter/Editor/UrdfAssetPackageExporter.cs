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
using UnityEditor;
using UnityEngine;

public static class UrdfAssetPackageExporter
{
    [MenuItem("Assets/Export URDF Asset Package",false,22)]
    private static void export()
    {
        List<string> urdfAssets = new List<string>();

        // Editor:
        urdfAssets.Add("Assets/UrdfImporter/Editor/ColladaImportAssetPostProcessor.cs");
        urdfAssets.Add("Assets/UrdfImporter/Editor/UrdfAssetPackageExporter.cs");
        urdfAssets.Add("Assets/UrdfImporter/Editor/UrdfCreatorEditorWindow.cs");
        urdfAssets.Add("Assets/UrdfImporter/Editor/UrdfImporterEditorWindow.cs");

        //  Plugins:
        urdfAssets.Add("Assets/UrdfImporter/Plugins/RosBridgeClient.dll");
        urdfAssets.Add("Assets/UrdfImporter/Plugins/Urdf.dll");

        // Scripts:
        urdfAssets.Add("Assets/UrdfImporter/Scripts/TransformExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfAssetDatabase.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfJointExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfLinkExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfLinkGeometryBoxExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfLinkGeometryCylinderExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfLinkGeometryExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfLinkGeometryMeshExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfLinkGeometrySphereExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfOriginExtensions.cs");
        urdfAssets.Add("Assets/UrdfImporter/Scripts/UrdfRobotExtensions.cs");

        string fileName = EditorUtility.SaveFilePanel(
                 "Save URDF Unity Asset Package",
                 Application.dataPath + "/../",
                 "UrdfImporter.unitypackage",
                 "unitypackage");

        if (fileName.Length == 0)
            return;

        AssetDatabase.ExportPackage(urdfAssets.ToArray(), fileName, ExportPackageOptions.Default);
    }
}
