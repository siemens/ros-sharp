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
    public class RenameRobotStateInWarehouseRequest : Message
    {
        public const string RosMessageName = "moveit_msgs/srv/RenameRobotStateInWarehouse";

        public string old_name { get; set; }
        public string new_name { get; set; }
        public string robot { get; set; }

        public RenameRobotStateInWarehouseRequest()
        {
            this.old_name = "";
            this.new_name = "";
            this.robot = "";
        }

        public RenameRobotStateInWarehouseRequest(string old_name, string new_name, string robot)
        {
            this.old_name = old_name;
            this.new_name = new_name;
            this.robot = robot;
        }
    }
}
#endif
