/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

#if ROS2

using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    public class Illuminance : Message
    {
        public const string RosMessageName = "sensor_msgs/msg/Illuminance";

        //  Single photometric illuminance measurement.  Light should be assumed to be
        //  measured along the sensor's x-axis (the area of detection is the y-z plane).
        //  The illuminance should have a 0 or positive value and be received with
        //  the sensor's +X axis pointing toward the light source.
        // 
        //  Photometric illuminance is the measure of the human eye's sensitivity of the
        //  intensity of light encountering or passing through a surface.
        // 
        //  All other Photometric and Radiometric measurements should not use this message.
        //  This message cannot represent:
        //   - Luminous intensity (candela/light source output)
        //   - Luminance (nits/light output per area)
        //   - Irradiance (watt/area), etc.
        public Header header { get; set; }
        // timestamp is the time the illuminance was measured
        //  frame_id is the location and direction of the reading
        public double illuminance { get; set; }
        // Measurement of the Photometric Illuminance in Lux.
        public double variance { get; set; }
        // 0 is interpreted as variance unknown

        public Illuminance()
        {
            this.header = new Header();
            this.illuminance = 0.0;
            this.variance = 0.0;
        }

        public Illuminance(Header header, double illuminance, double variance)
        {
            this.header = header;
            this.illuminance = illuminance;
            this.variance = variance;
        }
    }
}
#endif
