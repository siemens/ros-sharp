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
    public class DummyActionGoal : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/DummyActionGoal";

        public Header header;
        public GoalID goal_id;
        public DummyGoal goal;

        public DummyActionGoal()
        {
            header = new Header();
            goal_id = new GoalID();
            goal = new DummyGoal();
        }
    }
}

