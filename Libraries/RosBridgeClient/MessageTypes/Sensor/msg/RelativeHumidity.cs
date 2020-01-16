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

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    [DataContract]
    public class RelativeHumidity : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "sensor_msgs/RelativeHumidity";

        //  Single reading from a relative humidity sensor.  Defines the ratio of partial
        //  pressure of water vapor to the saturated vapor pressure at a temperature.
        [DataMember]
        public Header header;
        //  timestamp of the measurement
        //  frame_id is the location of the humidity sensor
        [DataMember]
        public double relative_humidity;
        //  Expression of the relative humidity
        //  from 0.0 to 1.0.
        //  0.0 is no partial pressure of water vapor
        //  1.0 represents partial pressure of saturation
        [DataMember]
        public double variance;
        //  0 is interpreted as variance unknown

        public RelativeHumidity()
        {
            this.header = new Header();
            this.relative_humidity = 0.0;
            this.variance = 0.0;
        }

        public RelativeHumidity(Header header, double relative_humidity, double variance)
        {
            this.header = header;
            this.relative_humidity = relative_humidity;
            this.variance = variance;
        }
    }
}
