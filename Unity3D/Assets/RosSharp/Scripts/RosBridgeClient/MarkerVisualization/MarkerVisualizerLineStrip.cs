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
    public class MarkerVisualizerLineStrip : MarkerVisualizer
    {
        private bool isCreated = false;

        List<GameObject> lines;

        private void Create()
        {
            markerObject = new GameObject("LineStrip");
            lines = new List<GameObject>();
            isCreated = true;
        }

        protected override void Visualize()
        {
            if (!isCreated)
                Create();

            markerObject.transform.SetParent(GameObject.Find(marker.header.frame_id).transform);

            markerObject.transform.localPosition = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.pose.position));
            markerObject.transform.localRotation = TransformExtensions.Ros2Unity(TypeExtensions.QuaternionMsgToQuaternion(marker.pose.orientation));

            if (marker.type == MessageTypes.Visualization.Marker.LINE_STRIP)
                LineList();
            else
                LineStrip();

            if (!marker.frame_locked)
                markerObject.transform.parent = null;
        }

        private void LineList()
        {
            while (lines.Count != marker.points.Length - 1)
            {
                if (lines.Count < marker.points.Length - 1)
                {
                    GameObject newObject = new GameObject("Line " + (lines.Count).ToString());
                    newObject.transform.SetParent(markerObject.transform, false);
                    LineRenderer renderer = newObject.AddComponent<LineRenderer>();
                    renderer.material = new Material(Shader.Find("Sprites/Default"));
                    renderer.positionCount = 2;
                    renderer.useWorldSpace = false;

                    lines.Add(newObject);
                }
                else
                {
                    lines.RemoveAt(lines.Count - 1);
                }
            }

            for (int i = 0; i < marker.points.Length - 1; i++)
            {
                LineRenderer renderer = lines[i].GetComponent<LineRenderer>();
                renderer.startWidth = (float)marker.scale.x;
                renderer.endWidth = (float)marker.scale.x;
                renderer.startColor = TypeExtensions.ColorRGBAToColor(marker.colors[i]);
                renderer.endColor = TypeExtensions.ColorRGBAToColor(marker.colors[i + 1]);
                renderer.SetPositions(new[] { TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[i])), TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[i + 1])) });
            }
        }

        private void LineStrip()
        {
            while (lines.Count != marker.points.Length / 2)
            {
                if (lines.Count < marker.points.Length / 2)
                {
                    GameObject newObject = new GameObject("Line " + (lines.Count).ToString());
                    newObject.transform.SetParent(markerObject.transform, false);
                    LineRenderer renderer = newObject.AddComponent<LineRenderer>();
                    renderer.material = new Material(Shader.Find("Sprites/Default"));
                    renderer.positionCount = 2;
                    renderer.useWorldSpace = false;

                    lines.Add(newObject);
                }
                else
                {
                    lines.RemoveAt(lines.Count - 1);
                }
            }

            for (int i = 0; i < marker.points.Length; i += 2)
            {
                LineRenderer renderer = lines[i / 2].GetComponent<LineRenderer>();
                renderer.startWidth = (float)marker.scale.x;
                renderer.endWidth = (float)marker.scale.x;
                renderer.startColor = TypeExtensions.ColorRGBAToColor(marker.colors[i]);
                renderer.endColor = TypeExtensions.ColorRGBAToColor(marker.colors[i + 1]);
                renderer.SetPositions(new[] { TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[i])), TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.points[i + 1])) });
            }

        }

        public override void DestroyObject()
        {
            base.DestroyObject();
            isCreated = false;
        }
    }
}