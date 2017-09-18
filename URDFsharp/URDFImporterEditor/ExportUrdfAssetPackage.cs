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


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


public static class ExportUrdfAssetPackage
{
    [MenuItem("ROSbridge/Export URDF Unity Asset Package ")]
    private static void Init()
    {
        List<string> assetPathNames = new List<string>();

        // Editor:
        assetPathNames.Add("Assets/Editor/Urdf/UrdfGeneratorEditorWindow.cs");
        assetPathNames.Add("Assets/Editor/Urdf/UrdfImporterEditorWindow.cs");
        assetPathNames.Add("Assets/Editor/Urdf/UrdfMeshImporter.cs");

        //  Plugins:
        assetPathNames.Add("Assets/Plugins/Newtonsoft.Json.dll");
        assetPathNames.Add("Assets/Plugins/RosBridgeClient.dll");
        assetPathNames.Add("Assets/Plugins/websocket-sharp.dll");

        // Scripts:
        assetPathNames.Add("Assets/Scripts/Urdf/Urdf.cs");

        // Extension Scripts:
        assetPathNames.Add("Assets/Scripts/Extensions/XAttributeExtensions.cs");
        assetPathNames.Add("Assets/Scripts/Extensions/TransformExtensions.cs");
        assetPathNames.Add("Assets/Scripts/Extensions/StringExtensions.cs");

        string fileName = EditorUtility.SaveFilePanel(
                 "Save URDF Unity Asset Package",
                 "",
                 "urdf.unitypackage",
                 "unitypackage");

        if (fileName.Length == 0)
            return;

        AssetDatabase.ExportPackage(assetPathNames.ToArray(), fileName, ExportPackageOptions.Default);
    }
}
