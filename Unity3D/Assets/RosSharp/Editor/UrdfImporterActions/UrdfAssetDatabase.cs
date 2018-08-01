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
        private static string assetPath;

        public static void Initialize(Robot robot)
        {
            assetPath = GetAssetParentDirectoryPath(robot.filename);

            if (!AssetDatabase.IsValidFolder(Path.Combine(assetPath, materialFolderName)))
                AssetDatabase.CreateFolder(assetPath, materialFolderName);

            createDefaultMaterialAsset();
            foreach (Link.Visual.Material material in robot.materials)
                CreateMaterialAsset(material);
        }

        #region SetAssetPath
        public static string GetAssetParentDirectoryPath(this string urdfFile)
        {
            string directoryAbsolutePath = Path.GetDirectoryName(urdfFile);
            return GetAssetPathFromAbsolutePath(directoryAbsolutePath);
        }

        public static void UpdateAssetPath(string newPath)
        {
            assetPath = newPath;
        }
        #endregion

        #region GetAssetPath
        public static string GetAssetRootFolder()
        {
            return assetPath;
        }

        public static string GetAssetPathFromPackagePath(string packagePath)
        {
            string path = packagePath.Substring(10).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);

            if (path.Substring(path.Length - 3, 3).ToLowerInvariant() == "stl")
                path = path.Substring(0, path.Length - 3) + "prefab";

            return Path.Combine(assetPath, path);
        }

        public static string GetAssetPathFromAbsolutePath(string absolutePath)
        {
            string absolutePathUnityFormat = absolutePath.Replace(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            if (absolutePathUnityFormat.StartsWith(Application.dataPath))
            {
                string assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
                return assetPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            }
            return null;
        }

        private static string getMaterialAssetPath(string materialName)
        {
            string path = Path.Combine(materialFolderName, Path.GetFileName(materialName) + ".mat");
            return Path.Combine(assetPath, path);
        }
        #endregion

        #region CreateMaterialAssets
        private static Material initializeMaterial()
        {
            Material material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);
            return material;
        }

        private static Material CreateMaterialAsset(this Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial.name == "")
            {
                Debug.LogWarning("Could not create a material without a name.");
                return null;
            }

            Material material = AssetDatabase.LoadAssetAtPath<Material>(getMaterialAssetPath(urdfMaterial.name));
            if (material != null) //material already exists
                return material;

            material = initializeMaterial();

            if (urdfMaterial.color != null)
                material.color = urdfMaterial.color.CreateColor();
            else if (urdfMaterial.texture != null)
                material.mainTexture = LoadTexture(urdfMaterial.texture.filename);

            AssetDatabase.CreateAsset(material, getMaterialAssetPath(urdfMaterial.name));
            return material;
        }

        private static Texture LoadTexture(string filename)
        {
            string path = Path.Combine(assetPath, filename).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            if (path == null)
                return null;
            return LocateAssetHandler.FindUrdfAsset<Texture>(path);
        }

        private static Material createDefaultMaterialAsset()
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(getMaterialAssetPath(defaultMaterialName));
            if (material != null)
                return material;

            material = initializeMaterial();
            material.color = new Color(0.33f, 0.33f, 0.33f, 0.0f);

            AssetDatabase.CreateAsset(material, getMaterialAssetPath(defaultMaterialName));
            return material;
        }
        #endregion CreateMaterialAssets

        #region SetMaterial
        public static void SetMaterial(GameObject gameObject, string materialName)
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(getMaterialAssetPath(materialName));
            Renderer[] renderers = gameObject.GetComponentsInChildren<Renderer>();
            foreach (Renderer renderer in renderers)
                renderer.sharedMaterial = material;
        }

        public static void SafelySetMaterial(GameObject gameObject, Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial == null)
                return;
            else if (urdfMaterial.name == "")
            {
                SetDefaultMaterial(gameObject);
                return;
            }

            Material material = AssetDatabase.LoadAssetAtPath<Material>(getMaterialAssetPath(urdfMaterial.name));
            if (material == null)
                CreateMaterialAsset(urdfMaterial);

            SetMaterial(gameObject, urdfMaterial.name);
        }

        public static void SetDefaultMaterial(GameObject gameObject)
        {
            SetMaterial(gameObject, defaultMaterialName);
        }
        #endregion
    }
}
