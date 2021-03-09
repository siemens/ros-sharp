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
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public class MarkerVisualizerMeshResource : MarkerVisualizer
    {
        private bool isCreated = false;

        private void Create()
        {
            markerObject = new GameObject("Mesh");
            markerObject.AddComponent<MeshFilter>();
            markerObject.AddComponent<MeshRenderer>();

            try
            {
                // Get name of mesh
                string[] s = marker.mesh_resource.Split('/');
                s = s.Last().Split('.');
                // Mesh needs a prefab in 'Assets\Resources\'
                GameObject meshObject = (GameObject)Resources.Load(s[0], typeof(GameObject));

                Mesh mesh = meshObject.GetComponentInChildren<MeshFilter>().sharedMesh;
                // Otherwise the object might be black
                mesh.RecalculateNormals();

                markerObject.GetComponent<MeshFilter>().mesh = mesh;
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
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

            markerObject.transform.localScale = TransformExtensions.Ros2UnityScale(TypeExtensions.Vector3MsgToVector3(marker.scale));

            markerObject.GetComponent<MeshRenderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.color));
        }

        public override void DestroyObject()
        {
            base.DestroyObject();
            isCreated = false;
        }
    }
}