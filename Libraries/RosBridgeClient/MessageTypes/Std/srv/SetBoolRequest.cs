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
    public class SetBoolRequest : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "std_srvs/SetBool";

        [DataMember]
        public bool data;
        //  e.g. for hardware enabling / disabling

        public SetBoolRequest()
        {
            this.data = false;
        }

        public SetBoolRequest(bool data)
        {
            this.data = data;
        }
    }
}
