/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if ROS2

namespace RosSharp.RosBridgeClient.MessageTypes.Rosapi
{
    public class ServiceResponseDetailsResponse : Message
    {
        public const string RosMessageName = "rosapi_msgs/srv/ServiceResponseDetails";

        public TypeDef[] typedefs { get; set; }

        public ServiceResponseDetailsResponse()
        {
            this.typedefs = new TypeDef[0];
        }

        public ServiceResponseDetailsResponse(TypeDef[] typedefs)
        {
            this.typedefs = typedefs;
        }
    }
}
#endif
