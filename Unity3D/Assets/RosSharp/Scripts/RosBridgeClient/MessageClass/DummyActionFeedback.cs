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
    public class DummyActionFeedback : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/DummyActionFeedback";

        public Header header;
        public GoalStatus status;
        public DummyFeedback feedback;

        public DummyActionFeedback()
        {
            header = new Header();
            status = new GoalStatus();
            feedback = new DummyFeedback();
        }
    }
}

