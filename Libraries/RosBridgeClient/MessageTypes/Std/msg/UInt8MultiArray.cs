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
    public class UInt8MultiArray : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "std_msgs/UInt8MultiArray";

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        [DataMember]
        public MultiArrayLayout layout;
        //  specification of data layout
        [DataMember]
        public byte[] data;
        //  array of data

        public UInt8MultiArray()
        {
            this.layout = new MultiArrayLayout();
            this.data = new byte[0];
        }

        public UInt8MultiArray(MultiArrayLayout layout, byte[] data)
        {
            this.layout = layout;
            this.data = data;
        }
    }
}
