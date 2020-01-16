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
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    [DataContract]
    public class Imu : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "sensor_msgs/Imu";

        //  This is a message to hold data from an IMU (Inertial Measurement Unit)
        // 
        //  Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
        // 
        //  If the covariance of the measurement is known, it should be filled in (if all you know is the 
        //  variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
        //  A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
        //  data a covariance will have to be assumed or gotten from some other source
        // 
        //  If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
        //  estimate), please set element 0 of the associated covariance matrix to -1
        //  If you are interpreting this message, please check for a value of -1 in the first element of each 
        //  covariance matrix, and disregard the associated estimate.
        [DataMember]
        public Header header;
        [DataMember]
        public Quaternion orientation;
        [DataMember]
        public double[] orientation_covariance;
        //  Row major about x, y, z axes
        [DataMember]
        public Vector3 angular_velocity;
        [DataMember]
        public double[] angular_velocity_covariance;
        //  Row major about x, y, z axes
        [DataMember]
        public Vector3 linear_acceleration;
        [DataMember]
        public double[] linear_acceleration_covariance;
        //  Row major x, y z 

        public Imu()
        {
            this.header = new Header();
            this.orientation = new Quaternion();
            this.orientation_covariance = new double[9];
            this.angular_velocity = new Vector3();
            this.angular_velocity_covariance = new double[9];
            this.linear_acceleration = new Vector3();
            this.linear_acceleration_covariance = new double[9];
        }

        public Imu(Header header, Quaternion orientation, double[] orientation_covariance, Vector3 angular_velocity, double[] angular_velocity_covariance, Vector3 linear_acceleration, double[] linear_acceleration_covariance)
        {
            this.header = header;
            this.orientation = orientation;
            this.orientation_covariance = orientation_covariance;
            this.angular_velocity = angular_velocity;
            this.angular_velocity_covariance = angular_velocity_covariance;
            this.linear_acceleration = linear_acceleration;
            this.linear_acceleration_covariance = linear_acceleration_covariance;
        }
    }
}
