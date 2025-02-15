/*
© Siemens AG, 2017-2019
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

#if UNITY_EDITOR

using UnityEngine;
using UnityEditor;
using System.Linq;
using System.IO;

namespace RosSharp
{
    public class StlAssetPostProcessor : AssetPostprocessor
    {
        private static void OnPostprocessAllAssets(string[] importedAssets, string[] deletedAssets, string[] movedAssets, string[] movedFromPath)
        {
            foreach (string stlFile in importedAssets.Where(x => x.ToLowerInvariant().EndsWith(".stl")))
                createStlPrefab(stlFile);
        }

        private static void createStlPrefab(string stlFile)
        {
            GameObject gameObject = CreateStlParent(stlFile);
            if (gameObject == null)
                return;

            PrefabUtility.SaveAsPrefabAsset(gameObject, getPrefabAssetPath(stlFile));
            Object.DestroyImmediate(gameObject);
        }

        private static GameObject CreateStlParent(string stlFile)
        {
            Mesh[] meshes = Urdf.StlImporter.ImportMesh(stlFile);
            if (meshes == null)
                return null;

            GameObject parent = new GameObject(Path.GetFileNameWithoutExtension(stlFile));
            Material material = AssetDatabase.GetBuiltinExtraResource<Material>("Default-Diffuse.mat");
            for (int i = 0; i < meshes.Length; i++)
            {
                string meshAssetPath = getMeshAssetPath(stlFile, i);
                AssetDatabase.CreateAsset(meshes[i], meshAssetPath);
                GameObject gameObject = CreateStlGameObject(meshAssetPath, material);
                gameObject.transform.SetParent(parent.transform, false);
            }
            return parent;
        }
        private static GameObject CreateStlGameObject(string meshAssetPath, Material material)
        {
            GameObject gameObject = new GameObject(Path.GetFileNameWithoutExtension(meshAssetPath));
            gameObject.AddComponent<MeshFilter>().sharedMesh = AssetDatabase.LoadAssetAtPath<Mesh>(meshAssetPath);
            gameObject.AddComponent<MeshRenderer>().sharedMaterial = material;
            return gameObject;
        }
        private static string getMeshAssetPath(string stlFile, int i)
        {
            return stlFile.Substring(0, stlFile.Length - 4) + "_" + i.ToString() + ".asset";
        }
        private static string getPrefabAssetPath(string stlFile)
        {
            return stlFile.Substring(0, stlFile.Length - 4) + ".prefab";
        }
    }
}

#endif