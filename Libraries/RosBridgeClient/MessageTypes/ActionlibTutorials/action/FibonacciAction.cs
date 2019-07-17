/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials
{
    public class FibonacciAction : Action<FibonacciActionGoal, FibonacciActionResult, FibonacciActionFeedback, FibonacciGoal, FibonacciResult, FibonacciFeedback>
    {
        [JsonIgnore]
        public const string RosMessageName = "actionlib_tutorials/FibonacciAction";

        public FibonacciAction() : base()
        {
            this.action_goal = new FibonacciActionGoal();
            this.action_result = new FibonacciActionResult();
            this.action_feedback = new FibonacciActionFeedback();
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
