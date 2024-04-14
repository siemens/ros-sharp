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
    public class MoveGroupGoal : Message
    {
        public const string RosMessageName = "moveit_msgs/action/MoveGroupGoal";

        //  Motion planning request to pass to planner
        public MotionPlanRequest request { get; set; }
        //  Planning options
        public PlanningOptions planning_options { get; set; }

        public MoveGroupGoal()
        {
            this.request = new MotionPlanRequest();
            this.planning_options = new PlanningOptions();
        }

        public MoveGroupGoal(MotionPlanRequest request, PlanningOptions planning_options)
        {
            this.request = request;
            this.planning_options = planning_options;
        }
    }
}
#endif
