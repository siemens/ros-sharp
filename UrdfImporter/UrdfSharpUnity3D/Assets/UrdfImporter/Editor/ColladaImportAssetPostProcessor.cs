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


using System.Xml.Linq;
using System.Globalization;
using UnityEditor;
using UnityEngine;
using System.IO;

public class ColladaImportAssetPostProcessor : AssetPostprocessor
{
    string orientation;

    public void OnPreprocessModel()
    {
        ModelImporter modelImporter = (ModelImporter)assetImporter;
        modelImporter.globalScale = readGlobalScale(getAbsolutePath(modelImporter.assetPath));
        modelImporter.animationType = ModelImporterAnimationType.None;
        orientation = readColladaOrientation(getAbsolutePath(modelImporter.assetPath));
    }

    public void OnPostprocessModel(GameObject gameObject)
    {
        gameObject.transform.Rotate(getColladaOrientationFix(orientation));
    }
    private static string getAbsolutePath(string relativeAssetPath)
    {
        return Path.Combine(Path.GetDirectoryName(Application.dataPath), relativeAssetPath);
    }

    private Vector3 getColladaOrientationFix(string orientation)
    {
        switch (orientation)
        {
            case "X_UP": return new Vector3(-90, 90, 90);//(-90, 90, 90);
            case "Y_UP": return new Vector3(-90, 90, 0);//(-90, 90, 0);
            case "Z_UP": return new Vector3(0, -90, 0);  //  fix
            default: return Vector3.zero;
        }
    }

    private string readColladaOrientation(string absolutePath)
    {
        try
        {
            XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
            XDocument xdoc = XDocument.Load(absolutePath);
            return xdoc.Element(xmlns + "COLLADA").Element(xmlns + "asset").Element(xmlns + "up_axis").Value;
        }
        catch
        {
            return "Z_UP";
        }
    }

    private float readGlobalScale(string absolutePath)
    {
        try
        {
            XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
            XDocument xdoc = XDocument.Load(absolutePath);
            string str = xdoc.Element(xmlns + "COLLADA").Element(xmlns + "asset").Element(xmlns + "unit").Attribute("meter").Value;
            return float.Parse(str, CultureInfo.InvariantCulture.NumberFormat);
        }
        catch
        {
            return 1.0f;
        }
    }
}
