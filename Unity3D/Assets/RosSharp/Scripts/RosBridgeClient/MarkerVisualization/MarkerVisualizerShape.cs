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

namespace RosSharp.RosBridgeClient
{
    public class MarkerVisualizerShape : MarkerVisualizer
    {
        private bool isCreated = false;

        private void Create()
        {
            switch (marker.type)
            {
                case (int)MessageTypes.Visualization.Marker.CUBE:
                    markerObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    break;
                case (int)MessageTypes.Visualization.Marker.SPHERE:
                    markerObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    break;
                case (int)MessageTypes.Visualization.Marker.CYLINDER:
                    markerObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    break;
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

            markerObject.transform.localScale = TransformExtensions.Ros2UnityScale(TypeExtensions.Vector3MsgToVector3(marker.scale));
            if (marker.type == MessageTypes.Visualization.Marker.CYLINDER)
                markerObject.transform.localScale = new Vector3(markerObject.transform.localScale.x, markerObject.transform.localScale.y / 2, markerObject.transform.localScale.z);

            markerObject.GetComponent<Renderer>().material.SetColor("_Color", TypeExtensions.ColorRGBAToColor(marker.color));

            if (!marker.frame_locked)
                markerObject.transform.parent = null;
        }

        public override void DestroyObject()
        {
            base.DestroyObject();
            isCreated = false;
        }
    }
}