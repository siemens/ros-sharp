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
    public class GetMapResult : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "nav_msgs/GetMapResult";

        //  ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
        public OccupancyGrid map;

        public GetMapResult()
        {
            this.map = new OccupancyGrid();
        }

        public GetMapResult(OccupancyGrid map)
        {
            this.map = map;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
