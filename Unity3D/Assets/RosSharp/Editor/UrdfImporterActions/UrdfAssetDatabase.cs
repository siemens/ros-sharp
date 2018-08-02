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

using System.IO;
using UnityEngine;
using UnityEditor;

namespace RosSharp.UrdfImporter
{
    public static class UrdfAssetDatabase
    {
        private const string defaultMaterialName = "Default";
        private const string materialFolderName = "Materials";
        private static string assetRootFolder;

        private static int unnamedMaterials = 0;

        public static void InitializeRobotAssets(Robot robot)
        {
            assetRootFolder = GetPathToParentDirectory(robot.filename);

            if (!AssetDatabase.IsValidFolder(Path.Combine(assetRootFolder, materialFolderName)))
                AssetDatabase.CreateFolder(assetRootFolder, materialFolderName);

            CreateDefaultMaterialAsset();
            foreach (Link.Visual.Material material in robot.materials)
                CreateMaterialAsset(material);
        }

        #region SetAssetRootFolder
        public static string GetPathToParentDirectory(this string urdfFile)
        {
            string directoryAbsolutePath = Path.GetDirectoryName(urdfFile);
            return GetRelativeAssetPath(directoryAbsolutePath);
        }

        public static void UpdateAssetRootFolder(string newPath)
        {
            assetRootFolder = newPath;
        }
        #endregion

        #region GetAssets
        public static string GetAssetRootFolder()
        {
            return assetRootFolder;
        }

        public static string GetRelativeAssetPath(string absolutePath)
        {
            string absolutePathUnityFormat = absolutePath.Replace(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            if (absolutePathUnityFormat.StartsWith(Application.dataPath))
            {
                string assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
                return assetPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            }
            return null;
        }

        private static string GetMaterialAssetPath(string materialName)
        {
            string path = Path.Combine(materialFolderName, Path.GetFileName(materialName) + ".mat");
            return Path.Combine(assetRootFolder, path);
        }
        #endregion

        #region CreateMaterialAssets
        private static Material CreateMaterialAsset(this Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial.name == "")
            {
                Debug.LogWarning("Invalid material name: cannot create a material without a name.");
                return null;
            }

            Material material = AssetDatabase.LoadAssetAtPath<Material>(GetMaterialAssetPath(urdfMaterial.name));
            if (material != null) //material already exists
                return material;

            material = InitializeMaterial();

            if (urdfMaterial.color != null)
                material.color = urdfMaterial.color.CreateColor();
            else if (urdfMaterial.texture != null)
                material.mainTexture = LoadTexture(urdfMaterial.texture.filename);

            AssetDatabase.CreateAsset(material, GetMaterialAssetPath(urdfMaterial.name));
            return material;
        }
        
        private static Material CreateDefaultMaterialAsset()
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(GetMaterialAssetPath(defaultMaterialName));
            if (material != null)
                return material;

            material = InitializeMaterial();
            material.color = new Color(0.33f, 0.33f, 0.33f, 0.0f);

            AssetDatabase.CreateAsset(material, GetMaterialAssetPath(defaultMaterialName));
            return material;
        }

        private static Material InitializeMaterial()
        {
            Material material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);
            return material;
        }

        private static Texture LoadTexture(string filename)
        {
            string path = Path.Combine(assetRootFolder, filename).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            if (path == null)
                return null;
            return LocateAssetHandler.FindUrdfAsset<Texture>(path);
        }
        #endregion CreateMaterialAssets

        #region SetMaterial
        public static void SafelySetMaterial(GameObject gameObject, Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial == null)
                return;
            else if (urdfMaterial.name == "")
                urdfMaterial.name = "material_" + unnamedMaterials++;

            Material material = AssetDatabase.LoadAssetAtPath<Material>(GetMaterialAssetPath(urdfMaterial.name));
            if (material == null)
                CreateMaterialAsset(urdfMaterial);

            SetMaterial(gameObject, urdfMaterial.name);
        }

        public static void SetDefaultMaterial(GameObject gameObject)
        {
            SetMaterial(gameObject, defaultMaterialName);
        }

        private static void SetMaterial(GameObject gameObject, string materialName)
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(GetMaterialAssetPath(materialName));
            Renderer[] renderers = gameObject.GetComponentsInChildren<Renderer>();
            foreach (Renderer renderer in renderers)
                renderer.sharedMaterial = material;
        }
        #endregion
    }
}
