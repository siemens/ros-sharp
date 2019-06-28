/*
© Siemens AG, 2017-2019
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
    public class JointState : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "sensor_msgs/JointState";
        public Standard.Header header;
        public string[] name;
        public double[] position;
        public double[] velocity;
        public double[] effort;

        public JointState()
        {
            header = new Standard.Header();
            name = new string[0];
            position = new double[0];
            velocity = new double[0];
            effort = new double[0];
        }
    }
}