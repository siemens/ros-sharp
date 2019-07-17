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

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    public class Temperature : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "sensor_msgs/Temperature";

        //  Single temperature reading.
        public Header header;
        //  timestamp is the time the temperature was measured
        //  frame_id is the location of the temperature reading
        public double temperature;
        //  Measurement of the Temperature in Degrees Celsius
        public double variance;
        //  0 is interpreted as variance unknown

        public Temperature()
        {
            this.header = new Header();
            this.temperature = 0.0;
            this.variance = 0.0;
        }

        public Temperature(Header header, double temperature, double variance)
        {
            this.header = header;
            this.temperature = temperature;
            this.variance = variance;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
