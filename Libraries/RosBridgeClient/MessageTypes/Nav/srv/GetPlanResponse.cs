/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

using RosSharp.RosBridgeClient.MessageTypes.Nav;

namespace RosSharp.RosBridgeClient.MessageTypes.Nav
{
    public class GetPlanResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "nav_msgs/GetPlan";

        public Path plan;

        public GetPlanResponse()
        {
            this.plan = new Path();
        }

        public GetPlanResponse(Path plan)
        {
            this.plan = plan;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
