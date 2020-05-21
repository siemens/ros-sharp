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
    public class PointCloud2 : Message
    { 
        public Standard.Header header;
        public uint height;
        public uint width;
        public PointField[] fields;
        public bool is_bigendian;
        public uint point_step;
        public uint row_step;

        public byte[] data;
        public bool is_dense;
        public PointCloud2()
        {
            header = new Standard.Header();
            height = 0;
            width = 0;
            fields = new PointField[0];
            is_bigendian = false;
            point_step = 0;
            row_step = 0;
            is_dense = false;
            data = new byte[0]; RosMessageName = "sensor_msgs/PointCloud2";
        }
    }
}