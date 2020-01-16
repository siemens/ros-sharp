/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

namespace RosSharp.RosBridgeClient.MessageTypes.Std
{
    [DataContract]
    public class UInt32 : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "std_msgs/UInt32";

        [DataMember]
        public uint data;

        public UInt32()
        {
            this.data = 0;
        }

        public UInt32(uint data)
        {
            this.data = data;
        }
    }
}
