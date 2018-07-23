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
using System;

namespace RosSharp.RosBridgeClient
{
    public class LaserScanSubscriber : Subscriber<Messages.Sensor.LaserScan>
    {
        private GameObject[] spheres;
        private Vector2[] spherePositions;
        private bool[] isInRange;
        private float maxRange;
        private float minRange;
        private bool isReceived = false;
        public GameObject laserScannerObject;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isReceived)
                ProcessMessage();

        }

        protected override void ReceiveMessage(Messages.Sensor.LaserScan laserScan)
        {
            if (spherePositions == null)
            {
                isInRange = new bool[laserScan.ranges.Length];
                spherePositions = new Vector2[laserScan.ranges.Length];
                maxRange = laserScan.range_max;
                minRange = laserScan.range_min;
            }

            for (int i = 0; i < laserScan.ranges.Length; i++)
            {
                isInRange[i] = laserScan.ranges[i] >= 0;
                spherePositions[i] = new Vector2(
                    (float)Math.Cos(laserScan.angle_min + i * laserScan.angle_increment) * laserScan.ranges[i],
                    (float)Math.Sin(laserScan.angle_min + i * laserScan.angle_increment) * laserScan.ranges[i]);
            }

            isReceived = true;
        }

        private void ProcessMessage()
        {
            if (spheres == null)
                InitializeSpheres(spherePositions.Length);

            if (spheres != null)
                for (int i = 0; i < spherePositions.Length; i++)
                {
                    spheres[i].SetActive(isInRange[i]);
                    spheres[i].transform.localPosition = new Vector3(spherePositions[i].x, spherePositions[i].y, 0).Ros2Unity();
                    spheres[i].transform.parent = laserScannerObject.transform;
                }
        }

        private void InitializeSpheres(int number)
        {
            spheres = new GameObject[number];

            for (int i = 0; i < number; i++)
                spheres[i] = InitializeSphere();
        }

        private GameObject InitializeSphere()
        {
            GameObject _gameObject = Instantiate(Resources.Load("LaserScanSphere", typeof(GameObject))) as GameObject;
            _gameObject.GetComponent<LaserScanWriter>().SetRanges(minRange, maxRange);
            _gameObject.transform.SetParentAndAlign(this.transform);
            return _gameObject;
        }

    }
}