using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    public static class UrdfMeshExportHandler {

        public static string CopyOrCreateMesh(GameObject geometryObject, bool isCollisionGeometry)
        {
            string prefabPath = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(geometryObject));
            bool foundExistingColladaOrStl = false;

            if (prefabPath != null && prefabPath != "")
            {
                if (prefabPath.ToLower().Contains(".dae"))
                    foundExistingColladaOrStl = true;
                else //Find STL file that corresponds to the prefab, if it already exists
                {
                    string[] foldersToSearch = { Path.GetDirectoryName(prefabPath) };
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
                          + geometryObject.name + ". Exporting a new STL file.", geometryObject);

                newFilePath = CreateNewStlFile(geometryObject, isCollisionGeometry);
            }

            return newFilePath;
        }

        private static string CopyAssetToExportDestination(string prefabPath)
        {
            string newPrefabPath = UrdfExportPathHandler.GetNewMeshPath(Path.GetFileName(prefabPath));
            newPrefabPath = newPrefabPath.SetSeparatorChar();

            prefabPath = UrdfAssetPathHandler.GetFullAssetPath(prefabPath);

            if (prefabPath != newPrefabPath) //Don't move prefab if it's already in the right place
                File.Copy(prefabPath, newPrefabPath, true);

            return newPrefabPath;
        }

        private static string CreateNewStlFile(GameObject geometryObject, bool isCollisionGeometry)
        {
            string newMeshPath = UrdfExportPathHandler.GetNewMeshPath(geometryObject.name + ".stl");

            StlExporter stlExporter = new StlExporter(newMeshPath, geometryObject, isCollisionGeometry);
            if (!stlExporter.Export())
                Debug.LogWarning("Mesh export for geometry " + geometryObject.name + " failed.", geometryObject);

            return newMeshPath;
        }
        
    }
}