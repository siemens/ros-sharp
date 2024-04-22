/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if ROS2

using RosSharp.RosBridgeClient.MessageTypes.Nav;

namespace RosSharp.RosBridgeClient.MessageTypes.Nav
{
    public class LoadMapResponse : Message
    {
        public const string RosMessageName = "nav_msgs/srv/LoadMap";

        //  Result code defintions
        public const byte RESULT_SUCCESS = 0;
        public const byte RESULT_MAP_DOES_NOT_EXIST = 1;
        public const byte RESULT_INVALID_MAP_DATA = 2;
        public const byte RESULT_INVALID_MAP_METADATA = 3;
        public const byte RESULT_UNDEFINED_FAILURE = 255;
        //  Returned map is only valid if result equals RESULT_SUCCESS
        public OccupancyGrid map { get; set; }
        public byte result { get; set; }

        public LoadMapResponse()
        {
            this.map = new OccupancyGrid();
            this.result = 0;
        }

        public LoadMapResponse(OccupancyGrid map, byte result)
        {
            this.map = map;
            this.result = result;
        }
    }
}
#endif