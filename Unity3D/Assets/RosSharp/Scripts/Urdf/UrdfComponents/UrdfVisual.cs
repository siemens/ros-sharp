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

using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

namespace RosSharp.Urdf.Export
{
    [SelectionBase]
    public class UrdfVisual : MonoBehaviour
    {
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
            
            string packagePath = "";
            string meshPath = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(geometryChildObject));

            bool foundExistingColladaOrStl = false;

            if (meshPath != null && meshPath != "")
            {
                if (meshPath.ToLower().Contains(".dae"))
                    foundExistingColladaOrStl = true;
                else
                {
                    //Find STL file, if already exists
                    string[] foldersToSearch = {Path.GetDirectoryName(meshPath)};
                    string meshName = Path.GetFileNameWithoutExtension(meshPath);

                    foreach (string guid2 in AssetDatabase.FindAssets(meshName, foldersToSearch))
                    {
                        if (AssetDatabase.GUIDToAssetPath(guid2).ToLower().Contains(".stl"))
                        {
                            meshPath = AssetDatabase.GUIDToAssetPath(guid2);
                            foundExistingColladaOrStl = true;
                            break;
                        }
                    }
                }
            }

            if (foundExistingColladaOrStl) //Copy prefab to new location in robot asset folder
            {
                string newMeshPath = UrdfAssetPathHandler.GetNewMeshPath(Path.GetFileName(meshPath));
                Directory.CreateDirectory(Path.GetDirectoryName(newMeshPath));

                packagePath = UrdfAssetPathHandler.GetPackagePathForMesh(meshPath);
                AssetDatabase.CopyAsset(meshPath, UrdfAssetPathHandler.GetRelativeAssetPath(newMeshPath));
            }
            else //Create new STL file 
            {
                Debug.Log("Did not find an existing STL or DAE file for Geometry Mesh " 
                          + geometryChildObject.name + ". Exporting a new STL file.");
    
                string newMeshPath = UrdfAssetPathHandler.GetNewMeshPath(geometryChildObject.name + ".STL");
                Directory.CreateDirectory(Path.GetDirectoryName(newMeshPath));

                packagePath = UrdfAssetPathHandler.GetPackagePathForMesh(newMeshPath);
                
                GameObject[] gameObjects = { geometryChildObject };
                StlExporter.Export(newMeshPath, gameObjects, FileType.Binary);
            }

            return new Link.Geometry(null, null, null, new Link.Geometry.Mesh(packagePath, transform.GetUrdfSize()));
        }

        private void CheckForUrdfIncompatibility()
        {
            Transform childTransform = transform.GetChild(0);
            if (childTransform != null &&
                (childTransform.localPosition != Vector3.zero || childTransform.localScale != Vector3.one))
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Visual object instead.");

            //Todo: test this
            if (transform.childCount > 1)
                Debug.LogWarning("Only one Geometry element is allowed for each Visual element. In link "
                                 + transform.parent.parent.name + ", move each Geometry into its own Visual element.");
        }
    }
}