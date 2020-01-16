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
    public class NodeDetailsResponse : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "rosapi/NodeDetails";

        [DataMember]
        public string[] subscribing;
        [DataMember]
        public string[] publishing;
        [DataMember]
        public string[] services;

        public NodeDetailsResponse()
        {
            this.subscribing = new string[0];
            this.publishing = new string[0];
            this.services = new string[0];
        }

        public NodeDetailsResponse(string[] subscribing, string[] publishing, string[] services)
        {
            this.subscribing = subscribing;
            this.publishing = publishing;
            this.services = services;
        }
    }
}
