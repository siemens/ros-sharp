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
    public class Image : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "sensor_msgs/Image";

        //  This message contains an uncompressed image
        //  (0, 0) is at top-left corner of image
        // 
        public Header header;
        //  Header timestamp should be acquisition time of image
        //  Header frame_id should be optical frame of camera
        //  origin of frame should be optical center of camera
        //  +x should point to the right in the image
        //  +y should point down in the image
        //  +z should point into to plane of the image
        //  If the frame_id here and the frame_id of the CameraInfo
        //  message associated with the image conflict
        //  the behavior is undefined
        public uint height;
        //  image height, that is, number of rows
        public uint width;
        //  image width, that is, number of columns
        //  The legal values for encoding are in file src/image_encodings.cpp
        //  If you want to standardize a new string format, join
        //  ros-users@lists.sourceforge.net and send an email proposing a new encoding.
        public string encoding;
        //  Encoding of pixels -- channel meaning, ordering, size
        //  taken from the list of strings in include/sensor_msgs/image_encodings.h
        public byte is_bigendian;
        //  is this data bigendian?
        public uint step;
        //  Full row length in bytes
        public byte[] data;
        //  actual matrix data, size is (step * rows)

        public Image()
        {
            this.header = new Header();
            this.height = 0;
            this.width = 0;
            this.encoding = "";
            this.is_bigendian = 0;
            this.step = 0;
            this.data = new byte[0];
        }

        public Image(Header header, uint height, uint width, string encoding, byte is_bigendian, uint step, byte[] data)
        {
            this.header = header;
            this.height = height;
            this.width = width;
            this.encoding = encoding;
            this.is_bigendian = is_bigendian;
            this.step = step;
            this.data = data;
        }

        public override string ToString()
        {
            return JsonConvert.SerializeObject(this);
        }
    }
}
