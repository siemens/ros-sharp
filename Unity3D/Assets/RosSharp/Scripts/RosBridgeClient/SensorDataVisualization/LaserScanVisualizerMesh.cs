/*
© Siemens AG, 2018
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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

namespace RosSharp.RosBridgeClient
{
    public class LaserScanVisualizerMesh : LaserScanVisualizer
    {
        public Material material;

        private GameObject LaserScanMesh;
        private Mesh mesh;
        private Vector3[] meshVerticies;
        private Color[] meshVertexColors;
        private int[] meshTriangles;
        private bool IsCreated = false;

        private void Create()
        {
            LaserScanMesh = new GameObject("LaserScanMesh");
            LaserScanMesh.transform.parent = null;

            LaserScanMesh.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = LaserScanMesh.AddComponent<MeshRenderer>();
            meshRenderer.material = material;

            mesh = LaserScanMesh.GetComponent<MeshFilter>().mesh;
            meshVerticies = new Vector3[directions.Length + 1];
            meshTriangles = new int[3 * (directions.Length - 1)];
            meshVertexColors = new Color[meshVerticies.Length];

            IsCreated = true;
        }

        protected override void Visualize()
        {
            if (!IsCreated)
                Create();

            LaserScanMesh.transform.SetPositionAndRotation(base_transform.position, base_transform.rotation);

            meshVerticies[0] = Vector3.zero;
            meshVertexColors[0] = Color.green;
            for (int i = 0; i < meshVerticies.Length - 1; i++)
            {
                meshVerticies[i + 1] = ranges[i] * directions[i];
                meshVertexColors[i + 1] = GetColor(ranges[i]);
            }
            for (int i = 0; i < meshTriangles.Length / 3; i++)
            {
                meshTriangles[3 * i] = 0;
                meshTriangles[3 * i + 1] = i + 2;
                meshTriangles[3 * i + 2] = i + 1;
            }

            mesh.vertices = meshVerticies;
            mesh.triangles = meshTriangles;
            mesh.colors = meshVertexColors;
        }

        protected override void DestroyObjects()
        {
            Destroy(LaserScanMesh);
            IsCreated = false;
        }
    }
}