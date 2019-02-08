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
    public class DummyGoal : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/DummyGoal";

        public Int32 order;
        public JointState joint_states;

        public DummyGoal()
        {
            order = new Int32();
            joint_states = new JointState();
        }
    }
}

