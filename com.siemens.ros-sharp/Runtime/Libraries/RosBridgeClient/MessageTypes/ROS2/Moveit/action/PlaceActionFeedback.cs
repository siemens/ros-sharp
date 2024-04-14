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
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient.MessageTypes.Moveit
{
    public class PlaceActionFeedback : ActionFeedback<PlaceFeedback>
    {
        public const string RosMessageName = "moveit_msgs/PlaceActionFeedback";

        public PlaceActionFeedback() : base()
        {
            this.feedback = new PlaceFeedback();
        }

        public PlaceActionFeedback(Header header, GoalStatus status, PlaceFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
    }
}
#endif
