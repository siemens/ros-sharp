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

namespace RosSharp.Urdf
{
    public static class UrdfAssetDatabase
    {
        private const string defaultMaterialName = "Default";
        private const string materialFolderName = "Materials";
        private static string assetPath;

        public static void Initialize(Robot robot)
        {
            assetPath = GetAssetPath(robot.filename);

            if (!AssetDatabase.IsValidFolder(Path.Combine(assetPath, materialFolderName)))
                AssetDatabase.CreateFolder(assetPath, materialFolderName);

            createDefaultMaterialAsset();
            foreach (Link.Visual.Material material in robot.materials)
                createMaterialAsset(material);
        }

        #region SetAssetPath
        public static string GetAssetPath(this string urdfFile)
        {
            string absolutePath = Path.GetDirectoryName(urdfFile);
            string absolutePathUnityFormat = absolutePath.Replace(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            if (absolutePathUnityFormat.StartsWith(Application.dataPath))
            {
                string assetPath = "Assets" + absolutePath.Substring(Application.dataPath.Length);
                return assetPath.Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            }
            return null;
        }
        #endregion

        #region GetAssetPath
        public static string GetAssetPathFromPackagePath(string packagePath)
        {
            string path = packagePath.Substring(10).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);

            if (path.Substring(path.Length - 3, 3).ToLowerInvariant() == "stl")
                path = path.Substring(0, path.Length - 3) + "prefab";

            return Path.Combine(assetPath, path);
        }

        private static string getMaterialAssetPath(string materialName)
        {
            string path = Path.Combine(materialFolderName, Path.GetFileName(materialName) + ".mat");
            return Path.Combine(assetPath,path);
        }
        #endregion

        #region CreateMaterialAssets
        private static Material initializeMaterial()        {
            Material material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);
            return material;
        }

        private static Material createMaterialAsset(this Link.Visual.Material urdfMaterial)
        {
            Material material = initializeMaterial();

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
            return AssetDatabase.LoadAssetAtPath<Texture>(path);
        }

        private static Material createDefaultMaterialAsset()
        {
            Material material = initializeMaterial();
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

        public static void SetDefaultMaterial(GameObject gameObject)
        {
            SetMaterial(gameObject, defaultMaterialName);
        }
        #endregion

    }
}
