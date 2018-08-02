﻿/*
© Siemens AG, 2018
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
using UnityEditor;
using UnityEngine;

namespace RosSharp.UrdfImporter
{
    public static class UrdfMaterialHandler
    {
        private const string defaultMaterialName = "Default";
        private const string materialFolderName = "Materials";
        private static int unnamedMaterials = 0;

        public static void InitializeRobotMaterials(Robot robot)
        {
            if (!AssetDatabase.IsValidFolder(Path.Combine(UrdfAssetPathHandler.GetAssetRootFolder(), materialFolderName)))
                AssetDatabase.CreateFolder(UrdfAssetPathHandler.GetAssetRootFolder(), materialFolderName);

            CreateDefaultMaterialAsset();
            foreach (Link.Visual.Material material in robot.materials)
                CreateMaterialAsset(material);
        }

        private static string GetMaterialAssetPath(string materialName)
        {
            string path = Path.Combine(materialFolderName, Path.GetFileName(materialName) + ".mat");
            return Path.Combine(UrdfAssetPathHandler.GetAssetRootFolder(), path);
        }

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
            string path = Path.Combine(UrdfAssetPathHandler.GetAssetRootFolder(), filename).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            if (path == null)
                return null;
            return LocateAssetHandler.FindUrdfAsset<Texture>(path);
        }
        #endregion CreateMaterialAssets

        #region SetMaterial
        public static void SetUrdfMaterial(GameObject gameObject, Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial != null && urdfMaterial.name == "")
            {
                //Assign a unique name to the material
                urdfMaterial.name = "material_" + unnamedMaterials++;
            }

            if (urdfMaterial != null)
            { 
                Material material = CreateMaterialAsset(urdfMaterial);
                SetMaterial(gameObject, material);
            }
            else
            {
                //If the URDF material is not defined, and the renderer is missing
                //a material, assign the default material.
                Renderer renderer = gameObject.GetComponentInChildren<Renderer>();
                if (renderer != null && renderer.sharedMaterial == null)
                    SetDefaultMaterial(gameObject);
            }
        }

        public static void SetDefaultMaterial(GameObject gameObject)
        {
            Material defaultMaterial = AssetDatabase.LoadAssetAtPath<Material>(GetMaterialAssetPath(defaultMaterialName));
            SetMaterial(gameObject, defaultMaterial);
        }

        private static void SetMaterial(GameObject gameObject, Material material)
        {
            Renderer[] renderers = gameObject.GetComponentsInChildren<Renderer>();
            foreach (Renderer renderer in renderers)
                renderer.sharedMaterial = material;
        }
        #endregion
    }
}
