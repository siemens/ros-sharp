/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if ROS2

namespace RosSharp.RosBridgeClient.MessageTypes.Moveit
{
    public class SaveMapRequest : Message
    {
        public const string RosMessageName = "moveit_msgs/srv/SaveMap";

        public string filename { get; set; }

        public SaveMapRequest()
        {
            this.filename = "";
        }

        public SaveMapRequest(string filename)
        {
            this.filename = filename;
        }
    }
}
#endif
