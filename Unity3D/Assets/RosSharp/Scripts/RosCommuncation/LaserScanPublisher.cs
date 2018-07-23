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

namespace RosSharp.RosBridgeClient
{
    public class LaserScanPublisher : Publisher<Messages.Sensor.LaserScan>
    {
        private Messages.Sensor.LaserScan message;        
        public string FrameId = "Unity";

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new Messages.Sensor.LaserScan
            {
                header = new Messages.Standard.Header { frame_id = FrameId },
                angle_min = 0,
                angle_max = 0,
                angle_increment = 0,
                time_increment = 0,
                range_min = 0,
                range_max = 0,
                ranges = new float[0],      // length?
                intensities = new float[0]
            };
        }

        private void UpdateMessage()
        {
            message.header.Update();
            UpdateLaserScan();
            Publish(message);
        }

        private void UpdateLaserScan()
        {
            // Call LaserScanReader and update LaserScan states  
        }
    }
}
