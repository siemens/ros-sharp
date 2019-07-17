/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.Rosapi
{
    public class SubscribersResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "rosapi/Subscribers";

        public string[] subscribers;

        public SubscribersResponse()
        {
            this.subscribers = new string[0];
        }

        public SubscribersResponse(string[] subscribers)
        {
            this.subscribers = subscribers;
        }
    }
}
