/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.Sensor
{
    public class LaserScan : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "sensor_msgs/LaserScan";
        public Standard.Header header;
        public float angle_min;
        public float angle_max;
        public float angle_increment;
        public float time_increment;
        public float range_min;
        public float range_max;
        public float[] ranges;
        public float[] intensities;

        public LaserScan()
        {
            header = new Standard.Header();
            angle_min = 0;
            angle_max = 0;
            angle_increment = 0;
            time_increment = 0;
            range_min = 0;
            range_max = 0;
            ranges = new float[0];
            intensities = new float[0];
        }
    }
}