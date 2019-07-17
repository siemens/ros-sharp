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
    public class Vector3Stamped : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "geometry_msgs/Vector3Stamped";

        //  This represents a Vector3 with reference coordinate frame and timestamp
        public Header header;
        public Vector3 vector;

        public Vector3Stamped()
        {
            this.header = new Header();
            this.vector = new Vector3();
        }

        public Vector3Stamped(Header header, Vector3 vector)
        {
            this.header = header;
            this.vector = vector;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
