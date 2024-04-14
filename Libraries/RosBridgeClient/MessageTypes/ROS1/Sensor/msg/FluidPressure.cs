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

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    public class FluidPressure : Message
    {
        public const string RosMessageName = "sensor_msgs/FluidPressure";

        //  Single pressure reading.  This message is appropriate for measuring the
        //  pressure inside of a fluid (air, water, etc).  This also includes
        //  atmospheric or barometric pressure.
        //  This message is not appropriate for force/pressure contact sensors.
        public Header header { get; set; }
        //  timestamp of the measurement
        //  frame_id is the location of the pressure sensor
        public double fluid_pressure { get; set; }
        //  Absolute pressure reading in Pascals.
        public double variance { get; set; }
        //  0 is interpreted as variance unknown

        public FluidPressure()
        {
            this.header = new Header();
            this.fluid_pressure = 0.0;
            this.variance = 0.0;
        }

        public FluidPressure(Header header, double fluid_pressure, double variance)
        {
            this.header = header;
            this.fluid_pressure = fluid_pressure;
            this.variance = variance;
        }
    }
}
#endif
