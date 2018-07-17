/*
© Siemens AG, 2017-2018
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
    public class LaserScanWriter : MonoBehaviour
    {
        private Renderer _renderer;
        private float _minRange;
        private float _maxRange;


        private void Start()
        {
            _renderer = GetComponent<Renderer>();
            //_renderer.material.shader = Shader.Find("Particles/Additive");
        }

        public void SetRanges(float minRange, float maxRange)
        {
            _minRange = minRange;
            _maxRange = maxRange;
        }

        private void Update()
        {
            UpdateColor(transform.localPosition.magnitude);
        }

        private void UpdateColor(float distance)
        {
            float h_min = (float)0;
            float h_max = (float)0.5;

            float h = (float)(h_min + (distance - _minRange) / (_maxRange - _minRange) * (h_max - h_min));
            float s = (float)1.0;
            float v = (float)1.0;

            _renderer.material.color = Color.HSVToRGB(h, s, v);
        }
    }
}