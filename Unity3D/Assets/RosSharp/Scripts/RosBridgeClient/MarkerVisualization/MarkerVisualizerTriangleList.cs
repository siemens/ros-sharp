/*
© Universität Hamburg, 2021
Author: Yannick Jonetzko (jonetzko@informatik.uni-hamburg.de)

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
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class MarkerVisualizerTriangleList : MarkerVisualizer
    {

        private bool isCreated = false;
        private List<GameObject> triangles;

        private void Create()
        {
            markerObject = new GameObject("MarkerTriangleList");
            triangles = new List<GameObject>();

            isCreated = true;
        }

        protected override void Visualize()
        {
            if (!isCreated)
                Create();

            markerObject.transform.SetParent(GameObject.Find(marker.header.frame_id).transform);

            markerObject.transform.localPosition = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.pose.position));
            markerObject.transform.localRotation = TransformExtensions.Ros2Unity(TypeExtensions.QuaternionMsgToQuaternion(marker.pose.orientation));

            if (!marker.frame_locked)
                markerObject.transform.parent = null;

            if (marker.points.Length != triangles.Count / 3)
                AdjustListLength();
            if (triangles.Count > 0)
                UpdateMarker();
        }

        public override void DestroyObject()
        {
            foreach (GameObject markerObject in triangles)
                Destroy(markerObject);
            base.DestroyObject();
            isCreated = false;
        }

        private void AdjustListLength()
        {
            while (triangles.Count != marker.points.Length / 3)
            {
                if (triangles.Count < marker.points.Length / 3)
                {
                    GameObject newObject = CreateMesh();
                    newObject.name = "Triangle " + (triangles.Count).ToString();
                    newObject.transform.SetParent(markerObject.transform, false);
                    triangles.Add(newObject);
                }
                else
                    triangles.RemoveAt(triangles.Count - 1);
            }
        }

        private void UpdateMarker()
        {
            for (int i = 0; i < triangles.Count; i++)
            {
                Mesh mesh = triangles[i].GetComponent<MeshFilter>().mesh;

                // update vertices
                mesh.SetVertices(new Vector3[] { TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[3 * i])),
                    TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[3 * i + 1])),
                    TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[3 * i + 2])) });
                // update color
                mesh.colors32 = (new Color32[] { TypeExtensions.ColorRGBAToColor(marker.colors[3 * i]),
                    TypeExtensions.ColorRGBAToColor(marker.colors[3 * i + 1]),
                    TypeExtensions.ColorRGBAToColor(marker.colors[3 * i + 2]) });
            }
        }

        private GameObject CreateMesh()
        {
            GameObject gameObject = new GameObject();
            Mesh mesh = new Mesh();

            // Vertices
            mesh.vertices = new Vector3[] { new Vector3(0, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 0, 0) };
            mesh.uv = new Vector2[] { new Vector2(0, 0), new Vector2(0, 1), new Vector2(1, 1) };
            mesh.triangles = new int[] { 0, 1, 2 };

            // Color
            gameObject.AddComponent<MeshRenderer>();
            Material material = new Material(Shader.Find("Particles/Standard Unlit"));
            // set two sided
            material.SetFloat("_Cull", 0);
            gameObject.GetComponent<MeshRenderer>().material = material;

            MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;

            return gameObject;
        }
    }
}