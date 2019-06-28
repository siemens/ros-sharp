/*
© Siemens AG, 2018-2019
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
    public class LaserScanVisualizerLines : LaserScanVisualizer
    {
        [Range(0.001f, 0.01f)]
        public float objectWidth;
        public Material material;

        private GameObject laserScanLines;
        private GameObject[] LaserScan;
        private bool IsCreated = false;

        private void Create(int numOfLines)
        {
            laserScanLines = new GameObject("laserScanLines");
            laserScanLines.transform.parent = null;

            LaserScan = new GameObject[numOfLines];
            for (int i = 0; i < numOfLines; i++)
            {
                LaserScan[i] = new GameObject("LaserScanLines");
                LaserScan[i].transform.parent = laserScanLines.transform;
                LaserScan[i].AddComponent<LineRenderer>();
                LaserScan[i].GetComponent<LineRenderer>().material = material;
            }
            IsCreated = true;
        }

        protected override void Visualize()
        {
            if (!IsCreated)
                Create(directions.Length);

            laserScanLines.transform.SetPositionAndRotation(base_transform.position, base_transform.rotation);

            for (int i = 0; i < directions.Length; i++)
            {
                LaserScan[i].transform.localPosition = ranges[i] * directions[i];
                LineRenderer lr = LaserScan[i].GetComponent<LineRenderer>();
                lr.startColor = GetColor(ranges[i]);
                lr.endColor = GetColor(ranges[i]);
                lr.startWidth = objectWidth;
                lr.endWidth = objectWidth;
                lr.useWorldSpace = false;
                lr.SetPosition(0, new Vector3(0, 0, 0));
                lr.SetPosition(1, -LaserScan[i].transform.localPosition);
            }
        }

        protected override void DestroyObjects()
        {
            for (int i = 0; i < LaserScan.Length; i++)
                Destroy(LaserScan[i]);

            Destroy(laserScanLines);
            IsCreated = false;
        }
    }
}