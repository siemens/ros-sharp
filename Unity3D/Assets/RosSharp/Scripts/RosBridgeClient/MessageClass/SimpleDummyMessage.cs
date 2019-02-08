/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Messages
{
    public class SimpleDummyMessage : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/SimpleDummyMessage";

        public Header header;
        public PoseWithCovariance pose;
        public Float64[] floatarray;
        public JointState[] jointstate;

        public SimpleDummyMessage()
        {
            header = new Header();
            pose = new PoseWithCovariance();
            floatarray = new Float64[0];
            jointstate = new JointState[0];
        }
    }
}

