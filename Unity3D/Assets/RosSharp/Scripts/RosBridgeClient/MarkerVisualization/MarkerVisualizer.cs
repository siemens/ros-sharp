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
    public abstract class MarkerVisualizer : MonoBehaviour
    {
        protected bool IsNewMarkerReceived = false;

        protected GameObject markerObject;
        public MessageTypes.Visualization.Marker marker;
        protected float lastUpdateTime;

        abstract protected void Visualize();

        protected void Update()
        {
            if (!IsNewMarkerReceived)
                return;

            IsNewMarkerReceived = false;
            Visualize();
        }

        protected void OnDisable()
        {
            DestroyObject();
        }

        public virtual void DestroyObject()
        {
            Destroy(markerObject);
        }

        public void SetMarkerData(MessageTypes.Visualization.Marker marker, float time)
        {
            this.marker = marker;
            lastUpdateTime = time;
            IsNewMarkerReceived = true;
        }

        public bool IsAlive()
        {
            if ((marker.lifetime.secs != 0 || marker.lifetime.nsecs != 0) && Time.time - lastUpdateTime > marker.lifetime.secs)
                return false;
            return true;
        }
    }
}