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

namespace Urdf
{
    public static class UrdfLinkGeometryBoxExtensions
    {
        public static GameObject CreateVisual(this Link.Geometry.Box box, GameObject parent)
        {
            GameObject gameObject = new GameObject("Box");
            gameObject.transform.SetParentAndAlign(parent.transform);

            MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
            meshFilter.mesh = box.GetMesh();
            gameObject.AddComponent<MeshRenderer>();

            return gameObject;
        }

        public static GameObject CreateCollider(this Link.Geometry.Box box, GameObject parent)
        {
            GameObject gameObject = new GameObject("Box");
            gameObject.transform.SetParentAndAlign(parent.transform);

            BoxCollider boxCollider = gameObject.AddComponent<BoxCollider>();
            boxCollider.size = new Vector3((float)box.size[1], (float)box.size[2], (float)box.size[0]);

            return gameObject;
        }

        private static Mesh GetMesh(this Link.Geometry.Box box)
        {
            float length = (float)box.size[1];
            float width = (float)box.size[2];
            float height = (float)box.size[0];

            #region Vertices
            Vector3 p0 = new Vector3(-length * .5f, -width * .5f, height * .5f);
            Vector3 p1 = new Vector3(length * .5f, -width * .5f, height * .5f);
            Vector3 p2 = new Vector3(length * .5f, -width * .5f, -height * .5f);
            Vector3 p3 = new Vector3(-length * .5f, -width * .5f, -height * .5f);

            Vector3 p4 = new Vector3(-length * .5f, width * .5f, height * .5f);
            Vector3 p5 = new Vector3(length * .5f, width * .5f, height * .5f);
            Vector3 p6 = new Vector3(length * .5f, width * .5f, -height * .5f);
            Vector3 p7 = new Vector3(-length * .5f, width * .5f, -height * .5f);

            Vector3[] vertices = new Vector3[]
            {
                p0, p1, p2, p3, // Bottom                
                p7, p4, p0, p3, // Left             
                p4, p5, p1, p0, // Front                
                p6, p7, p3, p2, // Back
                p5, p6, p2, p1, // Right
                p7, p6, p5, p4 // Top
            };
            #endregion

            #region Normales
            Vector3 up = Vector3.up;
            Vector3 down = Vector3.down;
            Vector3 front = Vector3.forward;
            Vector3 back = Vector3.back;
            Vector3 left = Vector3.left;
            Vector3 right = Vector3.right;

            Vector3[] normales = new Vector3[]
            {
                down, down, down, down, // Bottom
                left, left, left, left, // Left
                front, front, front, front, // Front
                back, back, back, back, // Back
                right, right, right, right, // Right
                up, up, up, up // Top
            };
            #endregion

            #region UVs
            Vector2 _00 = new Vector2(0f, 0f);
            Vector2 _10 = new Vector2(1f, 0f);
            Vector2 _01 = new Vector2(0f, 1f);
            Vector2 _11 = new Vector2(1f, 1f);

            Vector2[] uvs = new Vector2[]
            {
                _11, _01, _00, _10, // Bottom 
                _11, _01, _00, _10, // Left
                _11, _01, _00, _10, // Front
                _11, _01, _00, _10, // Back
                _11, _01, _00, _10, // Right
                _11, _01, _00, _10, // Top
            };
            #endregion

            #region Triangles
            int[] triangles = new int[]
            {
                3, 1, 0, // Bottom
                3, 2, 1,
                3 + 4 * 1, 1 + 4 * 1, 0 + 4 * 1, // Left
                3 + 4 * 1, 2 + 4 * 1, 1 + 4 * 1,
                3 + 4 * 2, 1 + 4 * 2, 0 + 4 * 2, // Front
                3 + 4 * 2, 2 + 4 * 2, 1 + 4 * 2,
                3 + 4 * 3, 1 + 4 * 3, 0 + 4 * 3, // Back
                3 + 4 * 3, 2 + 4 * 3, 1 + 4 * 3,
                3 + 4 * 4, 1 + 4 * 4, 0 + 4 * 4, // Right
                3 + 4 * 4, 2 + 4 * 4, 1 + 4 * 4,
                3 + 4 * 5, 1 + 4 * 5, 0 + 4 * 5, // Top
                3 + 4 * 5, 2 + 4 * 5, 1 + 4 * 5,
            };
            #endregion

            Mesh mesh = new Mesh();

            mesh.vertices = vertices;
            mesh.normals = normales;
            mesh.uv = uvs;
            mesh.triangles = triangles;

            mesh.RecalculateBounds();
            return mesh;
        }
    }
}