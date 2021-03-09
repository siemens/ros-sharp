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
    public class MarkerWriter : MonoBehaviour
    {
        protected bool isReceived = false;

        protected Dictionary<int, MarkerVisualizer> markerVisualizers;

        private MessageTypes.Visualization.Marker marker;

        private void Start()
        {
            markerVisualizers = new Dictionary<int, MarkerVisualizer>();
        }

        protected virtual void Update()
        {
            if (isReceived)
            {
                UpdateMarker(marker);
                if (markerVisualizers.Count > 0)
                    markerVisualizers[marker.id].SetMarkerData(marker, Time.time);
            }

            if (markerVisualizers.Count > 0)
                RemoveDeadMarker();
            isReceived = false;
        }

        public void Write(MessageTypes.Visualization.Marker marker)
        {
            this.marker = marker;
            isReceived = true;
        }

        protected void UpdateMarker(MessageTypes.Visualization.Marker marker)
        {
            if (marker.action == MessageTypes.Visualization.Marker.DELETEALL)
            {
                foreach (int key in markerVisualizers.Keys)
                    markerVisualizers[key].DestroyObject();
            }
            if (marker.action == MessageTypes.Visualization.Marker.DELETE)
                markerVisualizers[marker.id].DestroyObject();
            else if (!markerVisualizers.ContainsKey(marker.id)
                || markerVisualizers[marker.id].marker.action != marker.action
                || markerVisualizers[marker.id].marker.type != marker.type)
            {
                switch (marker.type)
                {
                    case (int)MessageTypes.Visualization.Marker.ARROW:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerArrow>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.CUBE:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerShape>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.SPHERE:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerShape>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.CYLINDER:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerShape>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.LINE_STRIP:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerLineStrip>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.LINE_LIST:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerLineStrip>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.CUBE_LIST:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerShapeList>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.SPHERE_LIST:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerShapeList>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.POINTS:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerShapeList>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.TEXT_VIEW_FACING:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerText>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.MESH_RESOURCE:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerMeshResource>());
                        break;
                    case (int)MessageTypes.Visualization.Marker.TRIANGLE_LIST:
                        markerVisualizers.Add(marker.id, gameObject.AddComponent<MarkerVisualizerTriangleList>());
                        break;
                    default:
                        Debug.LogError("Marker type not available");
                        break;
                }
            }
        }

        protected void RemoveDeadMarker()
        {
            List<int> deadMarkers = new List<int>();
            foreach (int key in markerVisualizers.Keys)
            {
                if (!markerVisualizers[key].IsAlive())
                    deadMarkers.Add(key);
            }
            foreach (int key in deadMarkers)
            {
                markerVisualizers[key].DestroyObject();
                markerVisualizers.Remove(key);
            }
        }
        protected void OnDisable()
        {
            foreach (int key in markerVisualizers.Keys)
                markerVisualizers[key].DestroyObject();
            markerVisualizers.Clear();
        }
    }
}