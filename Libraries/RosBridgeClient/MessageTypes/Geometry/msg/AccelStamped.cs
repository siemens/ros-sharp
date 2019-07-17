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
    public class AccelStamped : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "geometry_msgs/AccelStamped";

        //  An accel with reference coordinate frame and timestamp
        public Header header;
        public Accel accel;

        public AccelStamped()
        {
            this.header = new Header();
            this.accel = new Accel();
        }

        public AccelStamped(Header header, Accel accel)
        {
            this.header = header;
            this.accel = accel;
        }
    }
}
