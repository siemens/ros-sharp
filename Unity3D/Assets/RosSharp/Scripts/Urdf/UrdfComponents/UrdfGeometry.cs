using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    public class UrdfGeometry
    {
        public enum GeometryTypes { Box, Cylinder, Sphere, Mesh }

        public static GameObject GenerateVisualGeometry(GeometryTypes geometryType)
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
                    geometryGameObject = new GameObject();
                    geometryGameObject.AddComponent<MeshFilter>();
                    geometryGameObject.AddComponent<MeshRenderer>();
                    break;
            }
            return geometryGameObject;
        }

        public static GameObject GenerateCollisionGeometry(GeometryTypes geometryType)
        {
            GameObject geometryGameObject = new GameObject(geometryType.ToString());
            
            switch (geometryType)
            {
                case GeometryTypes.Box:
                    geometryGameObject.AddComponent<BoxCollider>();
                    break;
                case GeometryTypes.Cylinder:
                    geometryGameObject.AddComponent<CapsuleCollider>();
                    break;
                case GeometryTypes.Sphere:
                    geometryGameObject.AddComponent<SphereCollider>();
                    break;
                case GeometryTypes.Mesh:
                    geometryGameObject.AddComponent<MeshCollider>();
                    break;
            }

            return geometryGameObject;
        }

        public static Link.Geometry GetGeometryData(GeometryTypes geometryType, Transform transform, bool isCollisionGeometry = false)
        {
            Link.Geometry geometry = null;
            switch (geometryType)
            {
                case GeometryTypes.Box:
                    geometry = new Link.Geometry(new Link.Geometry.Box(transform.GetUrdfSize()));
                    break;
                case GeometryTypes.Cylinder:
                    geometry = new Link.Geometry(
                        null,
                        new Link.Geometry.Cylinder(transform.GetRadius(), transform.GetCylinderHeight()));
                    break;
                case GeometryTypes.Sphere:
                    geometry = new Link.Geometry(null, null, new Link.Geometry.Sphere(transform.GetRadius()));
                    break;
                case GeometryTypes.Mesh:
                    geometry = GetGeometryMeshData(transform.GetChild(0).gameObject, transform.GetUrdfSize(), isCollisionGeometry);
                    break;
            }

            return geometry;
        }


        public static Link.Geometry GetGeometryMeshData(GameObject geometryObject, double[] urdfSize, bool isCollisionGeometry)
        {
            string prefabPath = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(geometryObject));
            bool foundExistingColladaOrStl = false;

            if (prefabPath != null && prefabPath != "")
            {
                if (prefabPath.ToLower().Contains(".dae"))
                    foundExistingColladaOrStl = true;
                else //Find STL file that corresponds to the prefab, if it already exists
                {
                    string[] foldersToSearch =
                    {
                        Path.GetDirectoryName(prefabPath),
                        Path.Combine(UrdfAssetPathHandler.GetExportDestination(), "meshes")
                    };
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

            string newFilePath = "";
            if (foundExistingColladaOrStl)
                newFilePath = CopyAssetToExportDestination(prefabPath);
            else
            {
                Debug.Log("Did not find an existing STL or DAE file for Geometry Mesh "
                          + geometryObject.name + ". Exporting a new STL file.");

                newFilePath = CreateNewStlFile(geometryObject, isCollisionGeometry);
            }

            string packagePath = UrdfAssetPathHandler.GetPackagePathForMesh(newFilePath);

            return new Link.Geometry(null, null, null, new Link.Geometry.Mesh(packagePath, urdfSize));
        }

        private static string CopyAssetToExportDestination(string prefabPath)
        {
            string newPrefabPath = UrdfAssetPathHandler.GetNewMeshPath(Path.GetFileName(prefabPath));
            AssetDatabase.CopyAsset(prefabPath, newPrefabPath);

            return newPrefabPath;
        }

        private static string CreateNewStlFile(GameObject geometryObject, bool isCollisionGeometry)
        {
            //Create a clone with no rotation, so that it will be at original orientation when exported
            GameObject clone = Object.Instantiate(geometryObject, Vector3.zero, Quaternion.identity);
            clone.name = geometryObject.name;
            GameObject[] gameObjects = { clone };

            if (isCollisionGeometry)
            {
                //Add meshFilters so that collision meshes will be recognized by STL exporter
                foreach (MeshCollider meshCollider in clone.GetComponentsInChildren<MeshCollider>())
                {
                    MeshFilter meshFilter = meshCollider.gameObject.AddComponent<MeshFilter>();
                    meshFilter.sharedMesh = meshCollider.sharedMesh;
                }
            }

            string relativeMeshPath = UrdfAssetPathHandler.GetNewMeshPath(geometryObject.name + ".STL");
            StlExporter.Export(UrdfAssetPathHandler.GetFullAssetPath(relativeMeshPath), gameObjects, FileType.Binary);
            
            Object.DestroyImmediate(clone);

            return relativeMeshPath;
        }
    }
}