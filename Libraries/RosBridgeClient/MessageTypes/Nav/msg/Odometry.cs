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
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Nav
{
    public class Odometry : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "nav_msgs/Odometry";

        //  This represents an estimate of a position and velocity in free space.  
        //  The pose in this message should be specified in the coordinate frame given by header.frame_id.
        //  The twist in this message should be specified in the coordinate frame given by the child_frame_id
        public Header header;
        public string child_frame_id;
        public PoseWithCovariance pose;
        public TwistWithCovariance twist;

        public Odometry()
        {
            this.header = new Header();
            this.child_frame_id = "";
            this.pose = new PoseWithCovariance();
            this.twist = new TwistWithCovariance();
        }

        public Odometry(Header header, string child_frame_id, PoseWithCovariance pose, TwistWithCovariance twist)
        {
            this.header = header;
            this.child_frame_id = child_frame_id;
            this.pose = pose;
            this.twist = twist;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
