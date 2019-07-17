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

namespace RosSharp.RosBridgeClient.MessageTypes.Geometry
{
    public class PointStamped : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "geometry_msgs/PointStamped";

        //  This represents a Point with reference coordinate frame and timestamp
        public Header header;
        public Point point;

        public PointStamped()
        {
            this.header = new Header();
            this.point = new Point();
        }

        public PointStamped(Header header, Point point)
        {
            this.header = header;
            this.point = point;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
