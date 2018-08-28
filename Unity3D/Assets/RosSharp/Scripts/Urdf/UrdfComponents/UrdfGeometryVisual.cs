/*
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

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    class UrdfGeometryVisual : UrdfGeometry
    {
        public static GameObject Create(Transform parent, GeometryTypes geometryType, Link.Geometry geometry = null)
        {
            GameObject geometryGameObject = null;

            switch (geometryType)
            {
                case GeometryTypes.Box:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    geometryGameObject.transform.DestroyImmediateIfExists<BoxCollider>();
                    break;
                case GeometryTypes.Cylinder:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    geometryGameObject.transform.DestroyImmediateIfExists<CapsuleCollider>();
                    break;
                case GeometryTypes.Sphere:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    geometryGameObject.transform.DestroyImmediateIfExists<SphereCollider>();
                    break;
                case GeometryTypes.Mesh:
                    if (geometry != null)
                        geometryGameObject = CreateMeshVisual(parent, geometry.mesh);
                    //else, let user add their own mesh gameObject
                    break;
            }

            if (geometryGameObject != null)
            {
                geometryGameObject.transform.SetParentAndAlign(parent);
                if (geometry != null)
                    SetScale(parent, geometry, geometryType);
            }
            
            return geometryGameObject;
        }

        private static GameObject CreateMeshVisual(Transform parent, Link.Geometry.Mesh mesh)
        {
            GameObject meshObject = LocateAssetHandler.FindUrdfAsset<GameObject>(mesh.filename);
            return meshObject == null ? null : (GameObject)PrefabUtility.InstantiatePrefab(meshObject);
        }

        //public static GameObject CreateCylinderVisual(Link.Geometry.Cylinder cylinder, GameObject parent)
        //{
        //    GameObject gameObject = new GameObject("Cylinder");
        //    gameObject.transform.SetParentAndAlign(parent.transform);

        //    MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
        //    meshFilter.mesh = GetCylinderMesh(cylinder);
        //    gameObject.AddComponent<MeshRenderer>();

        //    return gameObject;
        //}
    }
}
