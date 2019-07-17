/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Rosapi
{
    public class GetTimeResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "rosapi/GetTime";

        public Time time;

        public GetTimeResponse()
        {
            this.time = new Time();
        }

        public GetTimeResponse(Time time)
        {
            this.time = time;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
