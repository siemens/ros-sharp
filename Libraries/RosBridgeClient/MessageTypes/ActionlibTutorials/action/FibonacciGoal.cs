/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

namespace RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials
{
    [DataContract]
    public class FibonacciGoal : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "actionlib_tutorials/FibonacciGoal";

        // goal definition
        [DataMember]
        public int order;

        public FibonacciGoal()
        {
            this.order = 0;
        }

        public FibonacciGoal(int order)
        {
            this.order = order;
        }
    }
}
