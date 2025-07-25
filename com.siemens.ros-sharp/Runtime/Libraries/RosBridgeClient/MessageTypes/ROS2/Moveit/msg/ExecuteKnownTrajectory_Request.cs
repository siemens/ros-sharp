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
    public class ExecuteKnownTrajectory_Request : Message
    {
        public const string RosMessageName = "moveit_msgs/msg/ExecuteKnownTrajectory_Request";

        //  This service is deprecated and will go away at some point. For new development use the ExecuteTrajectory action.
        //  Effective since: Indigo 0.7.4, Jade and Kinetic 0.8.3
        //  The trajectory to execute
        public RobotTrajectory trajectory { get; set; }
        //  Set this to true if the service should block until the execution is complete
        public bool wait_for_execution { get; set; }

        public ExecuteKnownTrajectory_Request()
        {
            this.trajectory = new RobotTrajectory();
            this.wait_for_execution = false;
        }

        public ExecuteKnownTrajectory_Request(RobotTrajectory trajectory, bool wait_for_execution)
        {
            this.trajectory = trajectory;
            this.wait_for_execution = wait_for_execution;
        }
    }
}
#endif
