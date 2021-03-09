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
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
    public class MarkerVisualizerText : MarkerVisualizer
    {
        private bool isCreated = false;
        private GameObject Canvas;
        private GameObject textObject;
        private Text text;

        private void Create()
        {
            markerObject = new GameObject("TextMarker");
            Canvas = new GameObject("Canvas");

            Canvas.AddComponent<Canvas>();
            Canvas.AddComponent<CanvasScaler>();
            Canvas.AddComponent<GraphicRaycaster>();

            Canvas.GetComponent<CanvasScaler>().dynamicPixelsPerUnit = 10;
            Canvas.GetComponent<RectTransform>().sizeDelta = new Vector2(2, 2);

            Canvas.transform.SetParent(markerObject.transform);

            textObject = new GameObject("Text");
            textObject.transform.SetParent(Canvas.transform);

            text = textObject.AddComponent<Text>();
            text.GetComponent<RectTransform>().sizeDelta = new Vector2(10000, 20);
            text.alignment = TextAnchor.MiddleCenter;

            Font ArialFont = (Font)Resources.GetBuiltinResource(typeof(Font), "Arial.ttf");
            text.font = ArialFont;
            text.material = ArialFont.material;

            isCreated = true;
        }

        protected override void Visualize()
        {
            if (!isCreated)
                Create();

            markerObject.transform.SetParent(GameObject.Find(marker.header.frame_id).transform);

            text.text = marker.text;
            text.color = TypeExtensions.ColorRGBAToColor(marker.color);

            float fontScale = 0.06f * (float)marker.scale.z;
            text.GetComponent<RectTransform>().localScale = new Vector3(fontScale, fontScale, fontScale);

            markerObject.transform.localPosition = TransformExtensions.Ros2Unity(TypeExtensions.PointMsgToVector3(marker.pose.position));
            // Look at camera
            markerObject.transform.rotation = Quaternion.LookRotation(markerObject.transform.position - Camera.main.transform.position);

            if (!marker.frame_locked)
                markerObject.transform.SetParent(null, true);
        }

        public override void DestroyObject()
        {
            base.DestroyObject();
            isCreated = false;
        }
    }
}