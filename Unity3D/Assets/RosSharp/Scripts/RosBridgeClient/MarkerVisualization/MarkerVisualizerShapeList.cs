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
    public class MarkerVisualizerShapeList : MarkerVisualizer
    {

        private bool isCreated = false;
        private List<GameObject> markerObjects;
        private Vector3 lookAt;

        private void Create()
        {
            markerObject = new GameObject("MarkerShapeList");
            markerObjects = new List<GameObject>();

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

            if (marker.points.Length != markerObjects.Count)
                AdjustListLength();
            if (markerObjects.Count > 0)
                UpdateMarker();
        }

        public override void DestroyObject()
        {
            foreach (GameObject markerObject in markerObjects)
                Destroy(markerObject);
            base.DestroyObject();
            isCreated = false;
        }

        private void AdjustListLength()
        {
            while (marker.points.Length != markerObjects.Count)
            {
                if (marker.points.Length > markerObjects.Count)
                {
                    GameObject newObject;
                    switch (marker.type)
                    {
                        case (int)MessageTypes.Visualization.Marker.CUBE_LIST:
                            newObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            newObject.name = "Cube " + markerObjects.Count.ToString();
                            break;
                        case (int)MessageTypes.Visualization.Marker.SPHERE_LIST:
                            newObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                            newObject.name = "Sphere " + markerObjects.Count.ToString();
                            break;
                        case (int)MessageTypes.Visualization.Marker.POINTS:
                            newObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            newObject.name = "Point " + markerObjects.Count.ToString();
                            break;
                        default:
                            Debug.LogError("Marker type not found.");
                            return;
                    }

                    newObject.transform.SetParent(markerObject.transform, true);
                    markerObjects.Add(newObject);
                }
                else
                    markerObjects.RemoveAt(markerObjects.Count - 1);
            }
        }

        private void UpdateMarker()
        {
            if (markerObjects.Count > 0)
            {
                lookAt = Camera.main.transform.position;
                lookAt.x = markerObjects[0].transform.position.x;
            }

            for (int i = 0; i < markerObjects.Count; i++)
            {
                markerObjects[i].transform.localPosition = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[i]));
                markerObjects[i].GetComponent<Renderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.colors[i]));

                if (marker.type == MessageTypes.Visualization.Marker.CUBE_LIST)
                {
                    markerObjects[i].transform.localRotation = TransformExtensions.Ros2Unity(TypeExtensions.QuaternionMsgToQuaternion(marker.pose.orientation));
                    markerObjects[i].transform.localScale = TransformExtensions.Ros2UnityScale(TypeExtensions.Vector3MsgToVector3(marker.scale));
                }
                if (marker.type == MessageTypes.Visualization.Marker.SPHERE_LIST || marker.type == MessageTypes.Visualization.Marker.POINTS)
                {
                    markerObjects[i].transform.localScale = new Vector3((float)marker.scale.x, (float)marker.scale.y, (float)marker.scale.y);
                    markerObjects[i].transform.LookAt(lookAt, markerObject.transform.right);
                }
            }
        }
    }
}