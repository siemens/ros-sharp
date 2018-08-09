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

using UnityEngine;
using UnityEditor;

namespace RosSharp.Urdf.Import
{
    public static class UrdfLinkGeometryMeshExtensions
    {
        public static GameObject CreateVisual(this Link.Geometry.Mesh mesh, GameObject parent)
        {
            GameObject meshObject = LocateAssetHandler.FindUrdfAsset<GameObject>(mesh.filename);

            if(meshObject != null)
            { 
                GameObject gameObject = Object.Instantiate(meshObject);
                mesh.setScale(gameObject);
                gameObject.transform.SetParentAndAlign(parent.transform);
                return gameObject;
            }
            return null;
        }

        public static GameObject CreateCollider(this Link.Geometry.Mesh mesh, GameObject parent)
        {
            GameObject prefabObject = LocateAssetHandler.FindUrdfAsset<GameObject>(mesh.filename);

            if (prefabObject == null)
                return null;
                    
            GameObject meshObject = Object.Instantiate(prefabObject);

            MeshFilter[] meshFilters = meshObject.GetComponentsInChildren<MeshFilter>();
            foreach (MeshFilter meshFilter in meshFilters)
            {
                GameObject child = meshFilter.gameObject;
                MeshCollider meshCollider = child.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = meshFilter.sharedMesh;
                Object.DestroyImmediate(child.GetComponent<MeshRenderer>());
                Object.DestroyImmediate(meshFilter);
            }

            mesh.setScale(meshObject);
            meshObject.transform.SetParentAndAlign(parent.transform);
            return meshObject;
        }

        private static void setScale(this Link.Geometry.Mesh mesh, GameObject gameObject)
        {
            if (mesh.scale != null)
            {
                Vector3 scale = new Vector3((float)mesh.scale[0], (float)mesh.scale[1], (float)mesh.scale[2]);
                gameObject.transform.localScale = Vector3.Scale(gameObject.transform.localScale, scale);
                gameObject.transform.localPosition = Vector3.Scale(gameObject.transform.localPosition, scale);
            }
        }
    }
}
