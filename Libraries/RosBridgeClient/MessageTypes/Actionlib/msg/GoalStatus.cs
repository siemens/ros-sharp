/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

namespace RosSharp.RosBridgeClient.MessageTypes.Actionlib
{
    [DataContract]
    public class GoalStatus : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "actionlib_msgs/GoalStatus";

        [DataMember]
        public GoalID goal_id;
        [DataMember]
        public byte status;
        [IgnoreDataMember]
        public const byte PENDING = 0; //  The goal has yet to be processed by the action server
        [IgnoreDataMember]
        public const byte ACTIVE = 1; //  The goal is currently being processed by the action server
        [IgnoreDataMember]
        public const byte PREEMPTED = 2; //  The goal received a cancel request after it started executing
        //    and has since completed its execution (Terminal State)
        [IgnoreDataMember]
        public const byte SUCCEEDED = 3; //  The goal was achieved successfully by the action server (Terminal State)
        [IgnoreDataMember]
        public const byte ABORTED = 4; //  The goal was aborted during execution by the action server due
        //     to some failure (Terminal State)
        [IgnoreDataMember]
        public const byte REJECTED = 5; //  The goal was rejected by the action server without being processed,
        //     because the goal was unattainable or invalid (Terminal State)
        [IgnoreDataMember]
        public const byte PREEMPTING = 6; //  The goal received a cancel request after it started executing
        //     and has not yet completed execution
        [IgnoreDataMember]
        public const byte RECALLING = 7; //  The goal received a cancel request before it started executing,
        //     but the action server has not yet confirmed that the goal is canceled
        [IgnoreDataMember]
        public const byte RECALLED = 8; //  The goal received a cancel request before it started executing
        //     and was successfully cancelled (Terminal State)
        [IgnoreDataMember]
        public const byte LOST = 9; //  An action client can determine that a goal is LOST. This should not be
        //     sent over the wire by an action server
        // Allow for the user to associate a string with GoalStatus for debugging
        [DataMember]
        public string text;

        public GoalStatus()
        {
            this.goal_id = new GoalID();
            this.status = 0;
            this.text = "";
        }

        public GoalStatus(GoalID goal_id, byte status, string text)
        {
            this.goal_id = goal_id;
            this.status = status;
            this.text = text;
        }
    }
}
