/*
This message class is generated automatically with 'ServiceMessageGenerator' of ROS#
*/

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Services
{
    public class AddTwoIntsRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "beginner_tutorials/AddTwoInts";

        public int a;
        public int b;

        public AddTwoIntsRequest(int _a, int _b)
        {
            a = _a;
            b = _b;
        }
    }

    public class AddTwoIntsResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "beginner_tutorials/AddTwoInts";

        public int sum;
    }
}

