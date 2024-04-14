/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if ROS2

using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Geometry
{
    public class PolygonStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/msg/PolygonStamped";

        //  This represents a Polygon with reference coordinate frame and timestamp
        public Header header { get; set; }
        public Polygon polygon { get; set; }

        public PolygonStamped()
        {
            this.header = new Header();
            this.polygon = new Polygon();
        }

        public PolygonStamped(Header header, Polygon polygon)
        {
            this.header = header;
            this.polygon = polygon;
        }
    }
}
#endif
