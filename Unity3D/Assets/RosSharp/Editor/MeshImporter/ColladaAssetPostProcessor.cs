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

namespace RosSharp
{
    public class ColladaAssetPostProcessor : AssetPostprocessor
    {
        private bool isCollada;
        private string orientation;

        public void OnPreprocessModel()
        {
            ModelImporter modelImporter = (ModelImporter)assetImporter;
            isCollada = Path.GetExtension(modelImporter.assetPath).ToLowerInvariant() == ".dae";

            if (!isCollada)
                return;

            modelImporter.globalScale = readGlobalScale(getAbsolutePath(modelImporter.assetPath));
            modelImporter.animationType = ModelImporterAnimationType.None;
            modelImporter.importCameras = false;
            modelImporter.importLights = false;
            orientation = readColladaOrientation(getAbsolutePath(modelImporter.assetPath));
        }

        public void OnPostprocessModel(GameObject gameObject)
        {
            if (!isCollada)
                return;

            gameObject.transform.SetPositionAndRotation(
                getColladaPositionFix(gameObject.transform.position, orientation),
                Quaternion.Euler(getColladaRotationFix(orientation)) * gameObject.transform.rotation);
        }

        private static string getAbsolutePath(string relativeAssetPath)
        {
            return Path.Combine(Path.GetDirectoryName(Application.dataPath), relativeAssetPath);
        }

        private Vector3 getColladaPositionFix(Vector3 position, string orientation)
        { 
            switch (orientation)
            {
                case "X_UP": return position; // not tested
                case "Y_UP": return position; // not tested
                case "Z_UP": return new Vector3(-position.z, position.y, -position.x); // tested
                default: return position; // not tested  
            }
        }

        private static Vector3 getColladaRotationFix(string orientation)
        {
            switch (orientation)
            { 
                case "X_UP": return new Vector3(-90, 90, 90); // not tested
                case "Y_UP": return new Vector3(-90, 90, 0);  // tested
                case "Z_UP": return new Vector3(0, 90, 0);    // tested
                default: return new Vector3(-90, 90, 0);    // tested                      
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
                return "undefined";
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
}