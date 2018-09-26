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

using UnityEngine;
using UnityEditor;

namespace RosSharp.Urdf.Editor
{
    public class UrdfGeometryCollision : UrdfGeometry
    {
        public static void Create(Transform parent, UrdfRobot.GeometryTypes geometryType, Link.Geometry geometry = null)
        {
            GameObject geometryGameObject = null;

            switch (geometryType)
            {
                case UrdfRobot.GeometryTypes.Box:
                    geometryGameObject = new GameObject(geometryType.ToString());
                    geometryGameObject.AddComponent<BoxCollider>();
                    break;
                case UrdfRobot.GeometryTypes.Cylinder:
                    geometryGameObject = CreateCylinderCollider();
                    break;
                case UrdfRobot.GeometryTypes.Sphere:
                    geometryGameObject = new GameObject(geometryType.ToString());
                    geometryGameObject.AddComponent<SphereCollider>();
                    break;
                case UrdfRobot.GeometryTypes.Mesh:
                    if (geometry != null)
                        geometryGameObject = CreateMeshCollider(geometry.mesh);
                    else
                    {
                        geometryGameObject = new GameObject(geometryType.ToString());
                        geometryGameObject.AddComponent<MeshCollider>();
                    }
                    break;
            }

            if(geometryGameObject != null)
            {
                geometryGameObject.transform.SetParentAndAlign(parent);
                if (geometry != null)
                    SetScale(parent, geometry, geometryType);
            }
        }

        private static GameObject CreateMeshCollider(Link.Geometry.Mesh mesh)
        {
            GameObject prefabObject = LocateAssetHandler.FindUrdfAsset<GameObject>(mesh.filename);

            if (prefabObject == null)
                return null;

            GameObject meshObject = (GameObject)PrefabUtility.InstantiatePrefab(prefabObject);
            ConvertMeshToColliders(meshObject);

            return meshObject;
        }

        private static GameObject CreateCylinderCollider()
        {
            GameObject gameObject = new GameObject("Cylinder");
            MeshCollider meshCollider = gameObject.AddComponent<MeshCollider>();

            Link.Geometry.Cylinder cylinder = new Link.Geometry.Cylinder(0.5, 2); //Default unity cylinder sizes

            meshCollider.sharedMesh = CreateCylinderMesh(cylinder);
            meshCollider.convex = true;

            return gameObject;
        }

        public static void CreateMatchingMeshCollision(Transform parent, Transform visualToCopy)
        {
            if (visualToCopy.childCount == 0) return;

            GameObject objectToCopy = visualToCopy.GetChild(0).gameObject;
            GameObject prefabObject = (GameObject)PrefabUtility.GetCorrespondingObjectFromSource(objectToCopy);

            GameObject collisionObject;
            if (prefabObject != null)
                collisionObject = (GameObject)PrefabUtility.InstantiatePrefab(prefabObject);
            else
                collisionObject = Object.Instantiate(objectToCopy);

            collisionObject.name = objectToCopy.name;
            ConvertMeshToColliders(collisionObject, true);

            collisionObject.transform.SetParentAndAlign(parent);
        }

        private static void ConvertMeshToColliders(GameObject gameObject, bool inflateColliders = false)
        {
            MeshFilter[] meshFilters = gameObject.GetComponentsInChildren<MeshFilter>();
            foreach (MeshFilter meshFilter in meshFilters)
            {
                GameObject child = meshFilter.gameObject;
                MeshCollider meshCollider = child.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = meshFilter.sharedMesh;

                if (inflateColliders)
                {
                    meshCollider.inflateMesh = true;
                    meshCollider.convex = true;
                    meshCollider.skinWidth = 0.001f;
                }

                Object.DestroyImmediate(child.GetComponent<MeshRenderer>());
                Object.DestroyImmediate(meshFilter);
            }
        }
    }
}
