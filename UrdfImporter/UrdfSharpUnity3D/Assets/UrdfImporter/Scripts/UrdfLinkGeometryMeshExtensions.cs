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

namespace Urdf
{
    public static class UrdfLinkGeometryMeshExtensions
    {
        public static GameObject CreateVisual(this Link.Geometry.Mesh mesh, GameObject parent)
        {
            GameObject gameObject = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(UrdfAssetDatabase.GetAssetPathFromPackagePath(mesh.filename)));
            gameObject.transform.SetParentAndAlign(parent.transform);
            mesh.setScale(gameObject);
            return gameObject;
        }

        public static GameObject CreateCollider(this Link.Geometry.Mesh mesh, GameObject parent)
        {
            GameObject gameObject = new GameObject(mesh.filename + "(MeshCollider)");
            Transform reference = AssetDatabase.LoadAssetAtPath<Transform>(UrdfAssetDatabase.GetAssetPathFromPackagePath(mesh.filename));
            gameObject.transform.position = reference.position;
            gameObject.transform.rotation = reference.rotation;
            gameObject.transform.localScale = reference.localScale;

            MeshCollider meshCollider = gameObject.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = AssetDatabase.LoadAssetAtPath<Mesh>(UrdfAssetDatabase.GetAssetPathFromPackagePath(mesh.filename));
            //meshCollider.convex = true;

            gameObject.transform.SetParentAndAlign(parent.transform);
            mesh.setScale(gameObject);

            return gameObject;
        }

        private static void setScale(this Link.Geometry.Mesh mesh, GameObject gameObject)
        {
            if (mesh.scale != null)
            {
                Vector3 scale = new Vector3((float)mesh.scale[0], (float)mesh.scale[1], (float)mesh.scale[2]);
                gameObject.transform.localScale = Vector3.Scale(gameObject.transform.localScale, scale);
            }

        }
    }
}