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
    public class SetBoolResponse : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "std_srvs/SetBool";

        [DataMember]
        public bool success;
        //  indicate successful run of triggered service
        [DataMember]
        public string message;
        //  informational, e.g. for error messages

        public SetBoolResponse()
        {
            this.success = false;
            this.message = "";
        }

        public SetBoolResponse(bool success, string message)
        {
            this.success = success;
            this.message = message;
        }
    }
}
