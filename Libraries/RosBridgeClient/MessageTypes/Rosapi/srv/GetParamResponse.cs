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
    public class GetParamResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "rosapi/GetParam";

        public string value;

        public GetParamResponse()
        {
            this.value = "";
        }

        public GetParamResponse(string value)
        {
            this.value = value;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
