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
    public class GetMotionPlanResponse : Message
    {
        public const string RosMessageName = "moveit_msgs/srv/GetMotionPlan";

        public MotionPlanResponse motion_plan_response { get; set; }

        public GetMotionPlanResponse()
        {
            this.motion_plan_response = new MotionPlanResponse();
        }

        public GetMotionPlanResponse(MotionPlanResponse motion_plan_response)
        {
            this.motion_plan_response = motion_plan_response;
        }
    }
}
#endif
