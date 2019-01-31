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
    public class DummyFeedback : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/DummyFeedback";

        public Image[] images;
        public Odometry odometry;
        public JointState joint_states;

        public DummyFeedback()
        {
            images = new Image[0];
            odometry = new Odometry();
            joint_states = new JointState();
        }
    }
}

