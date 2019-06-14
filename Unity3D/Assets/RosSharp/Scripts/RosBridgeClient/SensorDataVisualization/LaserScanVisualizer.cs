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
    public abstract class LaserScanVisualizer : MonoBehaviour
    {
        protected Transform base_transform;
        protected Vector3[] directions;
        protected float range_min;
        protected float range_max;
        protected float[] ranges;

        protected bool IsNewSensorDataReceived;
        protected bool IsVisualized = false;

        abstract protected void Visualize();
        abstract protected void DestroyObjects();

        protected void Update()
        {
            if (!IsNewSensorDataReceived)
                return;

            IsNewSensorDataReceived = false;
            Visualize();
        }

        protected void OnDisable()
        {
            DestroyObjects();
        }

        public void SetSensorData(Transform _base_transform, Vector3[] _directions, float[] _ranges, float _range_min, float _range_max)
        {
            base_transform = _base_transform;
            directions = _directions;
            ranges = _ranges;
            range_min = _range_min;
            range_max = _range_max;
            IsNewSensorDataReceived = true;
        }

        protected Color GetColor(float distance)
        {
            float h_min = (float)0;
            float h_max = (float)0.5;

            float h = (float)(h_min + (distance - range_min) / (range_max - range_min) * (h_max - h_min));
            float s = (float)1.0;
            float v = (float)1.0;

            return Color.HSVToRGB(h, s, v);
        }

    }
}