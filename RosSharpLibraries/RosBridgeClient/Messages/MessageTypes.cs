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

namespace RosSharp.RosBridgeClient.Messages
{
    public static class MessageTypes
    {
        public static readonly Dictionary<Type, string> Dictionary = new Dictionary<Type, string>
        {
            {typeof(GeometryTwist), "geometry_msgs/Twist"  },
            {typeof(StandardString), "std_msgs/String"  },
            {typeof(GeometryAccel), "geometry_msgs/Accel"  },
            {typeof(SensorJointStates), "sensor_msgs/JointState"  },
            {typeof(GeometryVector3),  "geometry_msgs/Vector3"},
            {typeof(SensorJoy),  "sensor_msgs/Joy"  },
            { typeof(NavigationOdometry), "nav_msgs/Odometry" },
            {typeof(StandardHeader), "std_msgs/Header"  },
            {typeof(GeometryPoseWithCovariance), "geometry_msgs/PoseWithCovariance" },
            { typeof(GeometryTwistWithCovariance),  "geometry_msgs/TwistWithCovariance" },
            {typeof(GeometryPose), "geometry_msgs/Pose"  },
            {typeof(GeometryPoseStamped) ,  "geometry_msgs/PoseStamped" },
            {typeof(GeometryPoint),  "geometry_msgs/Point"  },
            {typeof(GeometryQuaternion) ,  "geometry_msgs/Quaternion"},
            {typeof(SensorPointCloud2),  "sensor_msgs/PointCloud2" },
            {typeof(SensorPointField),  "sensor_msgs/PointField"  },
            {typeof(SensorImage), "sensor_msgs/Image"  },
            { typeof(SensorCompressedImage),  "sensor_msgs/CompressedImage" },
            {typeof(StandardTime),  "std_msgs/Time"    },
            { typeof(NavigationMapMetaData),  "nav_msgs/MapMetaData" },
            {typeof(NavigationOccupancyGrid),  "nav_msgs/OccupancyGrid" },
            {typeof(StandardServiceTriggerRequest), "std_srvs/Trigger" }
        };
        /*    public static string RosMessageType(Type messageType)
            {
                return Dictionary.FirstOrDefault(x => x.Value == messageType).Key;
            }
            public static Type MessageType(string rosMessageType)
            {
                Type messageType;
                Dictionary.TryGetValue(rosMessageType, out messageType);
                return messageType;
            }*/
    }
}
