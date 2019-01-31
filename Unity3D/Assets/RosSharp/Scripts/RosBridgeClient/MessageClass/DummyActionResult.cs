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
    public class DummyActionResult : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/DummyActionResult";

        public Header header;
        public GoalStatus status;
        public DummyResult result;

        public DummyActionResult()
        {
            header = new Header();
            status = new GoalStatus();
            result = new DummyResult();
        }
    }
}

