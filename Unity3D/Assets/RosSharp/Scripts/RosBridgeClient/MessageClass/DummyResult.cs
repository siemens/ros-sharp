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
    public class DummyResult : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "dummy_ros_pkg/DummyResult";

        public Image[] images;
        public LaserScan laser_scan;
        public Odometry odometry;

        public DummyResult()
        {
            images = new Image[0];
            laser_scan = new LaserScan();
            odometry = new Odometry();
        }
    }
}

