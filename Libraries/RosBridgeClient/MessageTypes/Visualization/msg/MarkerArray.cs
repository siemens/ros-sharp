﻿/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

namespace RosSharp.RosBridgeClient.MessageTypes.Visualization
{
    public class MarkerArray : Message
    {
        public const string RosMessageName = "visualization_msgs/MarkerArray";

        public Marker[] markers { get; set; }
        
        public MarkerArray()
        {
            markers = new Marker[0];
        }

        public MarkerArray(Marker[] markers)
        {
            this.markers = markers;
        }
    }
}