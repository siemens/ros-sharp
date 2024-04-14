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
    public class LocalPlannerActionGoal : ActionGoal<LocalPlannerGoal>
    {
        public const string RosMessageName = "moveit_msgs/LocalPlannerActionGoal";

        public LocalPlannerActionGoal() : base()
        {
            this.goal = new LocalPlannerGoal();
        }

        public LocalPlannerActionGoal(Header header, GoalID goal_id, LocalPlannerGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
    }
}
#endif
