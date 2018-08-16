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

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    [SelectionBase]
    public class UrdfVisual : MonoBehaviour
    {
        [SerializeField]
        private UrdfVisuals.GeometryTypes geometryType;

        public void Initialize(UrdfVisuals.GeometryTypes type)
        {
            GameObject geometryGameObject = null;
            geometryType = type;

            switch (geometryType)
            {
                case UrdfVisuals.GeometryTypes.Box:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    break;
                case UrdfVisuals.GeometryTypes.Cylinder:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    break;
                case UrdfVisuals.GeometryTypes.Sphere:
                    geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    break;
                case UrdfVisuals.GeometryTypes.Mesh:
                    geometryGameObject = new GameObject();
                    geometryGameObject.AddComponent<MeshFilter>();
                    geometryGameObject.AddComponent<MeshRenderer>();
                    break;
            }

            if (geometryGameObject == null) return;

            geometryGameObject.transform.SetParentAndAlign(gameObject.transform);
            transform.DestroyImmediateIfExists<Collider>();

            EditorGUIUtility.PingObject(gameObject);
        }

        public Link.Visual GetVisualData()
        {
            CheckForUrdfIncompatibility();

            Link.Geometry geometry = null;
            switch (geometryType)
            {
                case UrdfVisuals.GeometryTypes.Box:
                    geometry = new Link.Geometry(new Link.Geometry.Box(transform.GetUrdfSize()));
                    break;
                case UrdfVisuals.GeometryTypes.Cylinder:
                    geometry = new Link.Geometry(
                        null,
                        new Link.Geometry.Cylinder(transform.GetRadius(), transform.GetCylinderHeight()));
                    break;
                case UrdfVisuals.GeometryTypes.Sphere:
                    geometry = new Link.Geometry(null, null, new Link.Geometry.Sphere(transform.GetRadius()));
                    break;
                case UrdfVisuals.GeometryTypes.Mesh:
                    geometry = GetGeometryMeshData();
                    break;
            }
            
            Link.Visual.Material material = UrdfMaterial.GetMaterialData(gameObject.GetComponentInChildren<MeshRenderer>().sharedMaterial);
            string visualName = gameObject.name == "unnamed" ? null : gameObject.name;

            return new Link.Visual(geometry, visualName, transform.GetOriginData(), material);
        }

        public Link.Geometry GetGeometryMeshData()
        {
            GameObject geometryChildObject = transform.GetChild(0).gameObject;
            
            string prefabPath = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(geometryChildObject));
            bool foundExistingColladaOrStl = false;

            if (prefabPath != null && prefabPath != "")
            {
                if (prefabPath.ToLower().Contains(".dae"))
                    foundExistingColladaOrStl = true;
                else //Find STL file that corresponds to the prefab, if it already exists
                {
                    string[] foldersToSearch = {Path.GetDirectoryName(prefabPath)};
                    string prefabName = Path.GetFileNameWithoutExtension(prefabPath);

                    foreach (string guid2 in AssetDatabase.FindAssets(prefabName, foldersToSearch))
                    {
                        if (AssetDatabase.GUIDToAssetPath(guid2).ToLower().Contains(".stl"))
                        {
                            prefabPath = AssetDatabase.GUIDToAssetPath(guid2);
                            foundExistingColladaOrStl = true;
                            break;
                        }
                    }
                }
            }

            string packagePath = "";
            if (foundExistingColladaOrStl) //Copy prefab to new location in robot asset folder
            {
                string newPrefabPath = UrdfAssetPathHandler.GetNewMeshPath(Path.GetFileName(prefabPath));
                Directory.CreateDirectory(Path.GetDirectoryName(newPrefabPath));

                packagePath = UrdfAssetPathHandler.GetPackagePathForMesh(newPrefabPath);
                AssetDatabase.CopyAsset(prefabPath, UrdfAssetPathHandler.GetRelativeAssetPath(newPrefabPath));
            }
            else //Create new STL file 
            {
                string newPrefabPath = ExportMeshToStl(geometryChildObject);
                packagePath = UrdfAssetPathHandler.GetPackagePathForMesh(newPrefabPath);
            }

            return new Link.Geometry(null, null, null, new Link.Geometry.Mesh(packagePath, transform.GetUrdfSize()));
        }

        private static string ExportMeshToStl(GameObject gameObject)
        {
            Debug.Log("Did not find an existing STL or DAE file for Geometry Mesh "
                      + gameObject.name + ". Exporting a new STL file.");

            string newMeshPath = UrdfAssetPathHandler.GetNewMeshPath(gameObject.name + ".STL");
            Directory.CreateDirectory(Path.GetDirectoryName(newMeshPath));

            GameObject[] gameObjects = {gameObject};
            StlExporter.Export(newMeshPath, gameObjects, FileType.Binary);

            return newMeshPath;
        }

        private void CheckForUrdfIncompatibility()
        {
            Transform childTransform = transform.GetChild(0);
            if (childTransform != null &&
                (childTransform.localPosition != Vector3.zero || childTransform.localScale != Vector3.one))
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Visual object instead.");

            if (transform.childCount > 1)
                Debug.LogWarning("Only one Geometry element is allowed for each Visual element. One the first one will be exported to Urdf. In link "
                                 + transform.parent.parent.name + ", move each Geometry into its own Visual element.");
        }
    }
}