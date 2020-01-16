/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

namespace RosSharp.RosBridgeClient.MessageTypes.Geometry
{
    [DataContract]
    public class Wrench : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "geometry_msgs/Wrench";

        //  This represents force in free space, separated into
        //  its linear and angular parts.
        [DataMember]
        public Vector3 force;
        [DataMember]
        public Vector3 torque;

        public Wrench()
        {
            this.force = new Vector3();
            this.torque = new Vector3();
        }

        public Wrench(Vector3 force, Vector3 torque)
        {
            this.force = force;
            this.torque = torque;
        }
    }
}
