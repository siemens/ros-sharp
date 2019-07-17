/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    public class MultiDOFJointState : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "sensor_msgs/MultiDOFJointState";

        //  Representation of state for joints with multiple degrees of freedom, 
        //  following the structure of JointState.
        // 
        //  It is assumed that a joint in a system corresponds to a transform that gets applied 
        //  along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
        //  and those 3DOF can be expressed as a transformation matrix, and that transformation
        //  matrix can be converted back to (x, y, yaw)
        // 
        //  Each joint is uniquely identified by its name
        //  The header specifies the time at which the joint states were recorded. All the joint states
        //  in one message have to be recorded at the same time.
        // 
        //  This message consists of a multiple arrays, one for each part of the joint state. 
        //  The goal is to make each of the fields optional. When e.g. your joints have no
        //  wrench associated with them, you can leave the wrench array empty. 
        // 
        //  All arrays in this message should have the same size, or be empty.
        //  This is the only way to uniquely associate the joint name with the correct
        //  states.
        public Header header;
        public string[] joint_names;
        public Transform[] transforms;
        public Twist[] twist;
        public Wrench[] wrench;

        public MultiDOFJointState()
        {
            this.header = new Header();
            this.joint_names = new string[0];
            this.transforms = new Transform[0];
            this.twist = new Twist[0];
            this.wrench = new Wrench[0];
        }

        public MultiDOFJointState(Header header, string[] joint_names, Transform[] transforms, Twist[] twist, Wrench[] wrench)
        {
            this.header = header;
            this.joint_names = joint_names;
            this.transforms = transforms;
            this.twist = twist;
            this.wrench = wrench;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
