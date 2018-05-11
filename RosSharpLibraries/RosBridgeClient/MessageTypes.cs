/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Collections.Generic;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public static class MessageTypes
    {
        public static readonly Dictionary<string, Type> Dictionary = new Dictionary<string, Type>
        {
            { "geometry_msgs/Twist", typeof(GeometryTwist) },
            { "std_msgs/String", typeof(StandardString) },
            { "geometry_msgs/Accel", typeof(GeometryAccel) },
            { "sensor_msgs/JointState", typeof(SensorJointStates) },
            { "geometry_msgs/Vector3", typeof(GeometryVector3) },
            { "sensor_msgs/Joy", typeof(SensorJoy) },
            { "nav_msgs/Odometry", typeof(NavigationOdometry) },
            { "std_msgs/Header", typeof(StandardHeader) },
            { "geometry_msgs/PoseWithCovariance",typeof(GeometryPoseWithCovariance) },
            { "geometry_msgs/TwistWithCovariance", typeof(GeometryTwistWithCovariance) },
            { "geometry_msgs/Pose", typeof(GeometryPose) },
            { "geometry_msgs/PoseStamped", typeof(GeometryPoseStamped) },
            { "geometry_msgs/Point", typeof(GeometryPoint) },
            { "geometry_msgs/Quaternion",typeof(GeometryQuaternion) },
            { "sensor_msgs/PointCloud2",typeof(SensorPointCloud2) },
            { "sensor_msgs/PointField", typeof(SensorPointField) },
            { "sensor_msgs/Image", typeof(SensorImage) },
            { "sensor_msgs/CompressedImage", typeof(SensorCompressedImage) },
            { "std_msgs/Time", typeof(StandardTime)    },
            { "nav_msgs/MapMetaData", typeof(NavigationMapMetaData) },
            { "nav_msgs/OccupancyGrid", typeof(NavigationOccupancyGrid)}
        };
        public static string RosMessageType(Type messageType)
        {
            return Dictionary.FirstOrDefault(x => x.Value == messageType).Key;
        }
        public static Type MessageType(string rosMessageType)
        {
            Type messageType;
            Dictionary.TryGetValue(rosMessageType, out messageType);
            return messageType;
        }
    }
}
