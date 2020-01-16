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
    public class TopicsForTypeResponse : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "rosapi/TopicsForType";

        [DataMember]
        public string[] topics;

        public TopicsForTypeResponse()
        {
            this.topics = new string[0];
        }

        public TopicsForTypeResponse(string[] topics)
        {
            this.topics = topics;
        }
    }
}
