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

namespace RosSharp.RosBridgeClient.Messages
{
    public abstract class Message
    {
        public abstract string RosMessageName { get; }
        public static string GetRosName(System.Type t)
        {
            return (string)t.GetField("RosMessageName")?.GetRawConstantValue();
        }
    }

    public class GeometryTwist : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/Twist"; } }
        public GeometryVector3 linear;
        public GeometryVector3 angular;
        public GeometryTwist()
        {
            linear = new GeometryVector3();
            angular = new GeometryVector3();
        }
    }

    public class StandardString : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "std_msgs/String"; } }
        public string data;
        public StandardString(string data)
        {
            this.data = data;
        }
    }

    public class GeometryAccel : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/Accel"; } }
        public GeometryVector3 linear;
        public GeometryVector3 angular;
        public GeometryAccel()
        {
            linear = new GeometryVector3();
            angular = new GeometryVector3();
        }
    }

    public class SensorJointStates : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "sensor_msgs/JointState"; } }
        public StandardHeader header;
        public string[] name;
        public float[] position;
        public float[] velocity;
        public float[] effort;
        public SensorJointStates()
        {
            header = new StandardHeader();
            name = new string[0];
            position = new float[0];
            velocity = new float[0];
            effort = new float[0];
        }
    }
    public class GeometryVector3 : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/Vector3"; } }
        public float x;
        public float y;
        public float z;
        public GeometryVector3()
        {
            x = 0f;
            y = 0f;
            z = 0f;
        }
    }
    public class SensorJoy : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "sensor_msgs/Joy"; } }
        public StandardHeader header;
        public float[] axes;
        public int[] buttons;

        public SensorJoy()
        {
            header = new StandardHeader();
            axes = new float[0];
            buttons = new int[0];
        }
    }

    public class NavigationOdometry : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "nav_msgs/Odometry"; } }
        public StandardHeader header;
        public string child_frame_id;
        public GeometryPoseWithCovariance pose;
        public GeometryTwistWithCovariance twist;
        public NavigationOdometry()
        {
            header = new StandardHeader();
            child_frame_id = "";
            pose = new GeometryPoseWithCovariance();
            twist = new GeometryTwistWithCovariance();
        }
    }
    public class StandardHeader : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "std_msgs/Header"; } }
        public int seq;
        public StandardTime stamp;
        public string frame_id;
        public StandardHeader()
        {
            seq = 0;
            stamp = new StandardTime();
            frame_id = "";
        }
    }

    public class GeometryPoseWithCovariance : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/PoseWithCovariance"; } }
        public GeometryPose pose;
        public float[] covariance;
        public GeometryPoseWithCovariance()
        {
            pose = new GeometryPose();
            covariance = new float[36];
        }
    }
    public class GeometryTwistWithCovariance : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/TwistWithCovariance"; } }
        public GeometryTwist twist;
        public float[] covariance;
        public GeometryTwistWithCovariance()
        {
            twist = new GeometryTwist();
            covariance = new float[36];
        }
    }

    public class GeometryPose : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/Pose"; } }
        public GeometryPoint position;
        public GeometryQuaternion orientation;
        public GeometryPose()
        {
            position = new GeometryPoint();
            orientation = new GeometryQuaternion();
        }
    }

    public class GeometryPoseStamped : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/PoseStamped"; } }
        public StandardHeader header;
        public GeometryPose pose;
        public GeometryPoseStamped()
        {
            header = new StandardHeader();
            pose = new GeometryPose();
        }
    }

    public class GeometryPoint : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/Point"; } }
        public float x;
        public float y;
        public float z;
        public GeometryPoint()
        {
            x = 0;
            y = 0;
            z = 0;
        }
    }
    public class GeometryQuaternion : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "geometry_msgs/Quaternion"; } }
        public float x;
        public float y;
        public float z;
        public float w;
        public GeometryQuaternion()
        {
            x = 0;
            y = 0;
            z = 0;
            w = 0;
        }
    }
    public class SensorPointCloud2 : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "sensor_msgs/PointCloud2"; } }
        public StandardHeader header;
        public int height;
        public int width;
        public SensorPointField[] fields;
        public bool is_bigendian;
        public int point_step;
        public int row_step;

        public byte[] data;
        public bool is_dense;
        public SensorPointCloud2()
        {
            header = new StandardHeader();
            height = 0;
            width = 0;
            fields = new SensorPointField[0];
            is_bigendian = false;
            point_step = 0;
            row_step = 0;
            is_dense = false;
            data = new byte[0];
        }
    }
    public class SensorPointField : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "sensor_msgs/PointField"; } }
        public int datatype;
        public string name;
        public int offset;
        public int count;
        public SensorPointField()
        {
            datatype = 0;
            name = "";
            offset = 0;
            count = 0;
        }
    }
    public class SensorImage : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "sensor_msgs/Image"; } }
        public StandardHeader header;
        public int height;
        public int width;
        public string encoding;
        public bool is_bigendian;
        public int step;
        public byte[] data;
        public SensorImage()
        {
            header = new StandardHeader();
            height = 0;
            width = 0;
            encoding = "undefined";
            is_bigendian = false;
            step = 0;
            data = new byte[0];
        }
    }

    public class SensorCompressedImage : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "sensor_msgs/CompressedImage"; } }
        public StandardHeader header;
        public string format;
        public byte[] data;
        public SensorCompressedImage()
        {
            header = new StandardHeader();
            format = "";
            data = new byte[0];
        }
    }

    public class StandardTime : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "std_msgs/Time"; } }
        public int secs;
        public int nsecs;
        public StandardTime()
        {
            secs = 0;
            nsecs = 0;
        }
    }

    public class NavigationMapMetaData : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "nav_msgs/MapMetaData"; } }
        public StandardTime map_load_time;
        public float resolution;
        public uint width;
        public uint height;
        public GeometryPose origin;

        public NavigationMapMetaData()
        {
            map_load_time = null;
            resolution = 0;
            width = 0;
            height = 0;
            origin = new GeometryPose();
        }
    }

    public class NavigationOccupancyGrid : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "nav_msgs/OccupancyGrid"; } }
        public StandardHeader header;
        public NavigationMapMetaData info;
        public sbyte[] data;
        public NavigationOccupancyGrid()
        {
            header = new StandardHeader();
            info = new NavigationMapMetaData();
            data = null;
        }
    }
    public class StandardServiceTriggerRequest : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "std_srvs/Trigger"; } }
    }
    public class StandardServiceTriggerResponse : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "std_srvs/Trigger"; } }
        bool success;
        string message;
        public StandardServiceTriggerResponse(bool success, string message)
        {
            this.success = success;
            this.message = message;
        }
    }

    public class RosApiGetParamRequest : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "ros_api/GetParam"; } }
        public string name;
        public RosApiGetParamRequest(string name)
        {
            this.name = name;
        }
    }
    public class RosApiGetParamResponse : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "ros_api/GetParam"; } }
        public string value;
    }

    public class FileServerGetBinaryFileRequest : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "file_server/GetBinaryFile"; } }
        public string name;
        public FileServerGetBinaryFileRequest(string name)
        {
            this.name = name;
        }
    }
    public class FileServerGetBinaryFileResponse : Message
    {
        [JsonIgnore]
        public override string RosMessageName { get { return "file_server/GetBinaryFile"; } }
        public byte[] value;
    }
}
