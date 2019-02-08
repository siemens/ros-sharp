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
public class Request : Message
{
[JsonIgnore]
public const string RosMessageName = "/";

public Int32 a;
public Int32 b;

public Request(Int32 _a, Int32 _b){a = _a;
b = _b;
}
}

public class Response : Message
{
[JsonIgnore]
public const string RosMessageName = "/";

public Int32 sum;
}
}

