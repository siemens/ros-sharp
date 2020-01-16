/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using System.Runtime.Serialization;

namespace RosSharp.RosBridgeClient.MessageTypes.Shape
{
    [DataContract]
    public class MeshTriangle : Message
    {
        [IgnoreDataMember]
        public const string RosMessageName = "shape_msgs/MeshTriangle";

        //  Definition of a triangle's vertices
        [DataMember]
        public uint[] vertex_indices;

        public MeshTriangle()
        {
            this.vertex_indices = new uint[3];
        }

        public MeshTriangle(uint[] vertex_indices)
        {
            this.vertex_indices = vertex_indices;
        }
    }
}
