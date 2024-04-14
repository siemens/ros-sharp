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
    public class GetMotionSequence_Response : Message
    {
        public const string RosMessageName = "moveit_msgs/msg/GetMotionSequence_Response";

        //  Response to the planning request
        public MotionSequenceResponse response { get; set; }

        public GetMotionSequence_Response()
        {
            this.response = new MotionSequenceResponse();
        }

        public GetMotionSequence_Response(MotionSequenceResponse response)
        {
            this.response = response;
        }
    }
}
#endif
