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
using System;

namespace RosSharp.RosBridgeClient
{
    public class MarkerVisualizerArrow : MarkerVisualizer
    {
        private bool isCreated = false;
        private GameObject head;
        private GameObject shaft;
        private GameObject pivot;

        private void Create()
        {
            markerObject = new GameObject("Arrow");
            pivot = new GameObject("pivot");
            shaft = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            shaft.name = "Shaft";
            head = CreateHead(30, 1f, 1f);
            pivot.transform.SetParent(markerObject.transform);
            shaft.transform.SetParent(pivot.transform);

            head.transform.SetParent(pivot.transform);

            isCreated = true;
        }

        protected override void Visualize()
        {
            if (!isCreated)
                Create();

            markerObject.transform.SetParent(GameObject.Find(marker.header.frame_id).transform);

            markerObject.transform.localPosition = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.pose.position));
            markerObject.transform.localRotation = TransformExtensions.Ros2Unity(TypeExtensions.QuaternionMsgToQuaternion(marker.pose.orientation));

            head.GetComponent<Renderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.color));
            shaft.GetComponent<Renderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.color));

            float headLengthProportion = 0.23f;

            if (marker.points.Length == 0)
            {
                // Arrow shaft rotation doesn't match cylinder rotation in rviz...
                shaft.transform.localRotation = Quaternion.Euler(0, 90, 90);
                head.transform.localRotation = Quaternion.Euler(0, 90, 90);

                // head shaft ratios: 
                //    length 1:0.3
                //    width ratio is 2 to 1
                float shaftLength = (float)marker.scale.x * (1f - headLengthProportion);
                shaft.transform.localScale = TransformExtensions.Ros2UnityScale(new Vector3((float)marker.scale.y, (float)marker.scale.z, shaftLength));
                shaft.transform.localScale = new Vector3(shaft.transform.localScale.x, shaft.transform.localScale.y / 2, shaft.transform.localScale.z);
                shaft.transform.localPosition = new Vector3(shaft.transform.localPosition.x, shaft.transform.localPosition.y, shaft.transform.localScale.y);

                head.transform.localScale = TransformExtensions.Ros2UnityScale(new Vector3((float)marker.scale.y, (float)marker.scale.z, (float)marker.scale.x * headLengthProportion));
                head.transform.localPosition = new Vector3(head.transform.localPosition.x, head.transform.localPosition.y, shaft.transform.localScale.y * 2);
            }
            else
            {
                Vector3 point1 = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[0]));
                Vector3 point2 = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[1]));

                Vector3 direction = point2 - point1;
                float distance = direction.magnitude;
                float headLength = headLengthProportion * distance;

                if (marker.scale.z != 0.0)
                    headLength = Math.Max(0.0f, Math.Min((float)marker.scale.z, distance)); // clamp

                float shaftLength = distance - headLength;
                float shaftDiameter = (float)marker.scale.x;
                direction.Normalize();

                // scale and position arrow shaft
                shaft.transform.localScale = new Vector3(shaftDiameter, shaftLength / 2, shaftDiameter);
                shaft.transform.localPosition = transform.up * shaft.transform.localScale.y;

                // scale and position arrow head
                float headDiameter = (float)marker.scale.y * 0.5f;
                head.transform.localPosition = transform.up * shaft.transform.localScale.y * 2;
                head.transform.localScale = new Vector3(headDiameter, headLength, headDiameter);

                // rotate the arrow
                pivot.transform.localPosition = point1;
                Vector3 rotationAxis = Vector3.Normalize(direction + transform.up);
                pivot.transform.localRotation = new Quaternion(rotationAxis.x, rotationAxis.y, rotationAxis.z, 0);
            }

            if (!marker.frame_locked)
                markerObject.transform.parent = null;
        }

        public override void DestroyObject()
        {
            base.DestroyObject();
            Destroy(head);
            Destroy(shaft);
            isCreated = false;
        }

        private static GameObject CreateHead(int subdivisions, float radius, float height)
        {
            GameObject gameObject = new GameObject("Head");
            gameObject.AddComponent<MeshFilter>();
            gameObject.AddComponent<MeshRenderer>();

            Mesh mesh = new Mesh();

            Vector3[] vertices = new Vector3[subdivisions + 2];
            Vector2[] uv = new Vector2[vertices.Length];
            int[] triangles = new int[(subdivisions * 2) * 3];

            vertices[0] = Vector3.zero;
            uv[0] = new Vector2(0.5f, 0f);
            for (int i = 0, n = subdivisions - 1; i < subdivisions; i++)
            {
                float ratio = (float)i / n;
                float r = ratio * (Mathf.PI * 2f);
                float x = Mathf.Cos(r) * radius;
                float z = Mathf.Sin(r) * radius;
                vertices[i + 1] = new Vector3(x, 0f, z);

                uv[i + 1] = new Vector2(ratio, 0f);
            }
            vertices[subdivisions + 1] = new Vector3(0f, height, 0f);
            uv[subdivisions + 1] = new Vector2(0.5f, 1f);

            // construct bottom

            for (int i = 0, n = subdivisions - 1; i < n; i++)
            {
                int offset = i * 3;
                triangles[offset] = 0;
                triangles[offset + 1] = i + 1;
                triangles[offset + 2] = i + 2;
            }

            // construct sides

            int bottomOffset = subdivisions * 3;
            for (int i = 0, n = subdivisions - 1; i < n; i++)
            {
                int offset = i * 3 + bottomOffset;
                triangles[offset] = i + 1;
                triangles[offset + 1] = subdivisions + 1;
                triangles[offset + 2] = i + 2;
            }

            mesh.vertices = vertices;
            mesh.uv = uv;
            mesh.triangles = triangles;
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();

            gameObject.GetComponent<MeshFilter>().sharedMesh = mesh;

            return gameObject;
        }
    }
}