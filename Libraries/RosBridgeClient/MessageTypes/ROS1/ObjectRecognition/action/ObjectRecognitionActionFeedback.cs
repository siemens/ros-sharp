/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if !ROS2
using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient.MessageTypes.ObjectRecognition
{
    public class ObjectRecognitionActionFeedback : ActionFeedback<ObjectRecognitionFeedback>
    {
        public const string RosMessageName = "object_recognition_msgs/ObjectRecognitionActionFeedback";

        public ObjectRecognitionActionFeedback() : base()
        {
            this.feedback = new ObjectRecognitionFeedback();
        }

        public ObjectRecognitionActionFeedback(Header header, GoalStatus status, ObjectRecognitionFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
    }
}
#endif
