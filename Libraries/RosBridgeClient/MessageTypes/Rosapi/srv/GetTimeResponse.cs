/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Rosapi
{
    [DataContract]
    public class GetTimeResponse : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "rosapi/GetTime";

        [DataMember]
        public Time time;

        public GetTimeResponse()
        {
            this.time = new Time();
        }

        public GetTimeResponse(Time time)
        {
            this.time = time;
        }
    }
}
