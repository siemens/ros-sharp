//TODO: add header for pb_STL (https://github.com/karl-/pb_Stl)

using UnityEngine;
using System.Linq;
using System.Collections.Generic;

namespace RosSharp.Urdf
{
	/**
	 *	Provides menu items for writing STL files from a scene selection.
	 */
	public static class StlExporter
	{
		/**
		 * Export a hierarchy of GameObjects to path with file type.
		 */
		public static bool Export(string path, GameObject[] gameObjects, FileType type)
		{
            //TODO: fix so that meshes have the correct orientation when reimporting
			Mesh[] meshes = CreateWorldSpaceMeshesWithTransforms(gameObjects.Select(x => x.transform).ToArray());
			bool success = false;

			if(meshes != null && meshes.Length > 0)
			{
				if(!string.IsNullOrEmpty(path))
					success = StlWriter.WriteFile(path, meshes, type);
			}

			for(int i = 0; meshes != null && i < meshes.Length; i++)
				Object.DestroyImmediate(meshes[i]);

			return success;
		}

		/**
		 * Extracts a list of mesh values with their relative transformations intact.
		 */
		private static Mesh[] CreateWorldSpaceMeshesWithTransforms(IList<Transform> transforms)
		{
			if(transforms == null || transforms.Count < 1)
				return null;

			// move root node to center of selection
			Vector3 p = Vector3.zero;

			for(int i = 0; i < transforms.Count; i++)
				p += transforms[i].position;
			Vector3 mesh_center = p / (float) transforms.Count;

			GameObject root = new GameObject();
			root.name = "ROOT";
			root.transform.position = mesh_center;

			// copy all transforms to new root gameobject
			foreach(Transform t in transforms)
			{
				GameObject go = (GameObject) GameObject.Instantiate(t.gameObject);
				go.transform.SetParent(t.parent, false);
				go.transform.SetParent(root.transform, true);
			}

			// move root to 0,0,0 so mesh transformations are relative to origin
			root.transform.position = Vector3.zero;

			// create new meshes by iterating the root node and transforming vertex & normal
			// values (ignoring all other mesh attributes since STL doesn't care about them)
			List<MeshFilter> mfs = root.GetComponentsInChildren<MeshFilter>().Where(x => x.sharedMesh != null).ToList();
			int meshCount = mfs.Count;
			Mesh[] meshes = new Mesh[meshCount];

			for(int i = 0; i < meshCount; i++)
			{
				Transform t = mfs[i].transform;

				Vector3[] v = mfs[i].sharedMesh.vertices;
				Vector3[] n = mfs[i].sharedMesh.normals;

				for(int it = 0; it < v.Length; it++)
				{
					v[it] = t.TransformPoint(v[it]);
					n[it] = t.TransformDirection(n[it]);
				}

				Mesh m = new Mesh();

				m.name = mfs[i].name;
				m.vertices = v;
				m.normals = n;
				m.triangles = mfs[i].sharedMesh.triangles;

				meshes[i] = m;
			}

			// Cleanup
			GameObject.DestroyImmediate(root);

			return meshes;
		}
	}
}
