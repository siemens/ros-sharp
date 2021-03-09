using UnityEngine;

namespace RosSharp
{
    public static class TypeExtensions
    {
        public static Vector3 PointMsgToVector3(RosBridgeClient.MessageTypes.Geometry.Point pointMsg)
        {
            return new Vector3((float)pointMsg.x, (float)pointMsg.y, (float)pointMsg.z);
        }

        public static Vector3 Vector3MsgToVector3(RosBridgeClient.MessageTypes.Geometry.Vector3 vector3Msg)
        {
            return new Vector3((float)vector3Msg.x, (float)vector3Msg.y, (float)vector3Msg.z);
        }

        public static Quaternion QuaternionMsgToQuaternion(RosBridgeClient.MessageTypes.Geometry.Quaternion quaternionMsg)
        {
            return new Quaternion((float)quaternionMsg.x, (float)quaternionMsg.y, (float)quaternionMsg.z, (float)quaternionMsg.w);
        }

        public static Color ColorRGBAToColor(RosBridgeClient.MessageTypes.Std.ColorRGBA colorMsg)
        {
            return new Color(colorMsg.r, colorMsg.g, colorMsg.b, colorMsg.a);
        }
    }
}