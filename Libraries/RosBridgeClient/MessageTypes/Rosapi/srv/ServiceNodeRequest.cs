/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

namespace RosSharp.RosBridgeClient.MessageTypes.Rosapi
{
    [DataContract]
    public class ServiceNodeRequest : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "rosapi/ServiceNode";

        [DataMember]
        public string service;

        public ServiceNodeRequest()
        {
            this.service = "";
        }

        public ServiceNodeRequest(string service)
        {
            this.service = service;
        }
    }
}
