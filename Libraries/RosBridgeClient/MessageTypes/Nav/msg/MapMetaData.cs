/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Nav
{
    [DataContract]
    public class MapMetaData : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "nav_msgs/MapMetaData";

        //  This hold basic information about the characterists of the OccupancyGrid
        //  The time at which the map was loaded
        [DataMember]
        public Time map_load_time;
        //  The map resolution [m/cell]
        [DataMember]
        public float resolution;
        //  Map width [cells]
        [DataMember]
        public uint width;
        //  Map height [cells]
        [DataMember]
        public uint height;
        //  The origin of the map [m, m, rad].  This is the real-world pose of the
        //  cell (0,0) in the map.
        [DataMember]
        public Pose origin;

        public MapMetaData()
        {
            this.map_load_time = new Time();
            this.resolution = 0.0f;
            this.width = 0;
            this.height = 0;
            this.origin = new Pose();
        }

        public MapMetaData(Time map_load_time, float resolution, uint width, uint height, Pose origin)
        {
            this.map_load_time = map_load_time;
            this.resolution = resolution;
            this.width = width;
            this.height = height;
            this.origin = origin;
        }
    }
}
