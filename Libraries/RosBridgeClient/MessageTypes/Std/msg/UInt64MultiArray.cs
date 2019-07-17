/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.Std
{
    public class UInt64MultiArray : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "std_msgs/UInt64MultiArray";

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayout layout;
        //  specification of data layout
        public ulong[] data;
        //  array of data

        public UInt64MultiArray()
        {
            this.layout = new MultiArrayLayout();
            this.data = new ulong[0];
        }

        public UInt64MultiArray(MultiArrayLayout layout, ulong[] data)
        {
            this.layout = layout;
            this.data = data;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
