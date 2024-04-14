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
    public class GetPositionIK_Response : Message
    {
        public const string RosMessageName = "moveit_msgs/msg/GetPositionIK_Response";

        //  The returned solution 
        //  (in the same order as the list of joints specified in the IKRequest message)
        public RobotState solution { get; set; }
        public MoveItErrorCodes error_code { get; set; }

        public GetPositionIK_Response()
        {
            this.solution = new RobotState();
            this.error_code = new MoveItErrorCodes();
        }

        public GetPositionIK_Response(RobotState solution, MoveItErrorCodes error_code)
        {
            this.solution = solution;
            this.error_code = error_code;
        }
    }
}
#endif
