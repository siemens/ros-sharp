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
    public class MarkerArrayWriter : MarkerWriter
    {
        private MessageTypes.Visualization.Marker[] markerArray;

        protected override void Update()
        {
            if (isReceived)
            {
                foreach (MessageTypes.Visualization.Marker marker in markerArray)
                {
                    UpdateMarker(marker);
                    markerVisualizers[marker.id].SetMarkerData(marker, Time.time);
                }
            }
            if (markerVisualizers.Count > 0)
                RemoveDeadMarker();
            isReceived = false;
        }

        public void Write(MessageTypes.Visualization.MarkerArray markerArray)
        {
            this.markerArray = markerArray.markers;
            isReceived = true;
        }
    }
}