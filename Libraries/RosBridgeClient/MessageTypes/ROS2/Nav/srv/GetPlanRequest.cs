/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if ROS2

using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Nav
{
    public class GetPlanRequest : Message
    {
        public const string RosMessageName = "nav_msgs/srv/GetPlan";

        //  Get a plan from the current position to the goal Pose
        //  The start pose for the plan
        public PoseStamped start { get; set; }
        //  The final pose of the goal position
        public PoseStamped goal { get; set; }
        //  If the goal is obstructed, how many meters the planner can
        //  relax the constraint in x and y before failing.
        public float tolerance { get; set; }

        public GetPlanRequest()
        {
            this.start = new PoseStamped();
            this.goal = new PoseStamped();
            this.tolerance = 0.0f;
        }

        public GetPlanRequest(PoseStamped start, PoseStamped goal, float tolerance)
        {
            this.start = start;
            this.goal = goal;
            this.tolerance = tolerance;
        }
    }
}
#endif
