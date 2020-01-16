#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 168

namespace Utf8Json.Resolvers
{
    using System;
    using Utf8Json;

    public class RosSharpResolverGen : global::Utf8Json.IJsonFormatterResolver
    {
        public static readonly global::Utf8Json.IJsonFormatterResolver Instance = new RosSharpResolverGen();

        RosSharpResolverGen()
        {

        }

        public global::Utf8Json.IJsonFormatter<T> GetFormatter<T>()
        {
            return FormatterCache<T>.formatter;
        }

        static class FormatterCache<T>
        {
            public static readonly global::Utf8Json.IJsonFormatter<T> formatter;

            static FormatterCache()
            {
                var f = RosSharpResolverGenGetFormatterHelper.GetFormatter(typeof(T));
                if (f != null)
                {
                    formatter = (global::Utf8Json.IJsonFormatter<T>)f;
                }
            }
        }
    }

    internal static class RosSharpResolverGenGetFormatterHelper
    {
        static readonly global::System.Collections.Generic.Dictionary<Type, int> lookup;

        static RosSharpResolverGenGetFormatterHelper()
        {
            lookup = new global::System.Collections.Generic.Dictionary<Type, int>(197)
            {
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus[]), 0 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]), 1 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose[]), 2 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]), 3 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped[]), 4 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]), 5 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback[]), 6 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]), 7 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]), 8 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench[]), 9 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]), 10 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32[]), 11 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField[]), 12 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle[]), 13 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension[]), 14 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped[]), 15 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint[]), 16 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint[]), 17 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration), 18 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time), 19 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID), 20 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus), 21 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header), 22 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusArray), 23 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal), 24 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal), 25 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult), 26 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult), 27 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback), 28 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback), 29 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciAction), 30 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileRequest), 31 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileResponse), 32 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileRequest), 33 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileResponse), 34 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3), 35 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel), 36 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelStamped), 37 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance), 38 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceStamped), 39 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia), 40 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaStamped), 41 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point), 42 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32), 43 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PointStamped), 44 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon), 45 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonStamped), 46 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion), 47 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose), 48 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose2D), 49 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseArray), 50 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped), 51 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance), 52 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped), 53 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionStamped), 54 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform), 55 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped), 56 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist), 57 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistStamped), 58 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance), 59 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceStamped), 60 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3Stamped), 61 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench), 62 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchStamped), 63 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal), 64 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData), 65 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid), 66 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult), 67 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult), 68 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback), 69 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapAction), 70 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GridCells), 71 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.Odometry), 72 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path), 73 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResponse), 74 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanRequest), 75 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanResponse), 76 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapRequest), 77 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapResponse), 78 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef), 79 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.DeleteParamRequest), 80 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetActionServersResponse), 81 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamNamesResponse), 82 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamRequest), 83 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamResponse), 84 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetTimeResponse), 85 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamRequest), 86 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamResponse), 87 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsRequest), 88 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsResponse), 89 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsRequest), 90 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsResponse), 91 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodesResponse), 92 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersRequest), 93 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersResponse), 94 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamRequest), 95 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamResponse), 96 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostRequest), 97 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostResponse), 98 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeRequest), 99 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeResponse), 100 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersRequest), 101 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersResponse), 102 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsRequest), 103 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsResponse), 104 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsRequest), 105 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsResponse), 106 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeRequest), 107 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeResponse), 108 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesResponse), 109 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeRequest), 110 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeResponse), 111 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SetParamRequest), 112 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersRequest), 113 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersResponse), 114 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeRequest), 115 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeResponse), 116 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsResponse), 117 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeRequest), 118 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeResponse), 119 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.BatteryState), 120 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest), 121 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo), 122 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32), 123 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage), 124 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.FluidPressure), 125 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Illuminance), 126 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Image), 127 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Imu), 128 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState), 129 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Joy), 130 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback), 131 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackArray), 132 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho), 133 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScan), 134 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MagneticField), 135 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiDOFJointState), 136 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiEchoLaserScan), 137 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus), 138 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatFix), 139 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud), 140 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField), 141 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud2), 142 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Range), 143 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RelativeHumidity), 144 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Temperature), 145 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.TimeReference), 146 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoRequest), 147 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoResponse), 148 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle), 149 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Shape.Mesh), 150 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Shape.Plane), 151 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Shape.SolidPrimitive), 152 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Bool), 153 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Byte), 154 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension), 155 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout), 156 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.ByteMultiArray), 157 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Char), 158 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.ColorRGBA), 159 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32), 160 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray), 161 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64), 162 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64MultiArray), 163 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16), 164 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16MultiArray), 165 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32), 166 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray), 167 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64), 168 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64MultiArray), 169 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8), 170 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8MultiArray), 171 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.String), 172 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16), 173 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16MultiArray), 174 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32), 175 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32MultiArray), 176 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64), 177 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64MultiArray), 178 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8), 179 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8MultiArray), 180 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolRequest), 181 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolResponse), 182 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerResponse), 183 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal), 184 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal), 185 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error), 186 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult), 187 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult), 188 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback), 189 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformAction), 190 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TFMessage), 191 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.FrameGraphResponse), 192 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint), 193 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectory), 194 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint), 195 },
                {typeof(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectory), 196 },
            };
        }

        internal static object GetFormatter(Type t)
        {
            int key;
            if (!lookup.TryGetValue(t, out key)) return null;

            switch (key)
            {
                case 0: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>();
                case 1: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32>();
                case 2: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>();
                case 3: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point>();
                case 4: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped>();
                case 5: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef>();
                case 6: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback>();
                case 7: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform>();
                case 8: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>();
                case 9: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench>();
                case 10: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho>();
                case 11: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32>();
                case 12: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField>();
                case 13: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle>();
                case 14: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension>();
                case 15: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped>();
                case 16: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint>();
                case 17: return new global::Utf8Json.Formatters.ArrayFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint>();
                case 18: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.DurationFormatter();
                case 19: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.TimeFormatter();
                case 20: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalIDFormatter();
                case 21: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusFormatter();
                case 22: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.HeaderFormatter();
                case 23: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusArrayFormatter();
                case 24: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoalFormatter();
                case 25: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoalFormatter();
                case 26: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResultFormatter();
                case 27: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResultFormatter();
                case 28: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedbackFormatter();
                case 29: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedbackFormatter();
                case 30: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFormatter();
                case 31: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileRequestFormatter();
                case 32: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileResponseFormatter();
                case 33: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileRequestFormatter();
                case 34: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileResponseFormatter();
                case 35: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3Formatter();
                case 36: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelFormatter();
                case 37: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelStampedFormatter();
                case 38: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceFormatter();
                case 39: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceStampedFormatter();
                case 40: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaFormatter();
                case 41: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaStampedFormatter();
                case 42: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PointFormatter();
                case 43: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32Formatter();
                case 44: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PointStampedFormatter();
                case 45: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonFormatter();
                case 46: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonStampedFormatter();
                case 47: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionFormatter();
                case 48: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseFormatter();
                case 49: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose2DFormatter();
                case 50: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseArrayFormatter();
                case 51: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStampedFormatter();
                case 52: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceFormatter();
                case 53: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStampedFormatter();
                case 54: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionStampedFormatter();
                case 55: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformFormatter();
                case 56: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStampedFormatter();
                case 57: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistFormatter();
                case 58: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistStampedFormatter();
                case 59: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceFormatter();
                case 60: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceStampedFormatter();
                case 61: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3StampedFormatter();
                case 62: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchFormatter();
                case 63: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchStampedFormatter();
                case 64: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoalFormatter();
                case 65: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaDataFormatter();
                case 66: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGridFormatter();
                case 67: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResultFormatter();
                case 68: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResultFormatter();
                case 69: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedbackFormatter();
                case 70: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFormatter();
                case 71: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GridCellsFormatter();
                case 72: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.OdometryFormatter();
                case 73: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.PathFormatter();
                case 74: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResponseFormatter();
                case 75: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanRequestFormatter();
                case 76: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanResponseFormatter();
                case 77: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapRequestFormatter();
                case 78: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapResponseFormatter();
                case 79: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDefFormatter();
                case 80: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.DeleteParamRequestFormatter();
                case 81: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetActionServersResponseFormatter();
                case 82: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamNamesResponseFormatter();
                case 83: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamRequestFormatter();
                case 84: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamResponseFormatter();
                case 85: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetTimeResponseFormatter();
                case 86: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamRequestFormatter();
                case 87: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamResponseFormatter();
                case 88: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsRequestFormatter();
                case 89: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsResponseFormatter();
                case 90: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsRequestFormatter();
                case 91: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsResponseFormatter();
                case 92: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodesResponseFormatter();
                case 93: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersRequestFormatter();
                case 94: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersResponseFormatter();
                case 95: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamRequestFormatter();
                case 96: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamResponseFormatter();
                case 97: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostRequestFormatter();
                case 98: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostResponseFormatter();
                case 99: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeRequestFormatter();
                case 100: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeResponseFormatter();
                case 101: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersRequestFormatter();
                case 102: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersResponseFormatter();
                case 103: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsRequestFormatter();
                case 104: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsResponseFormatter();
                case 105: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsRequestFormatter();
                case 106: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsResponseFormatter();
                case 107: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeRequestFormatter();
                case 108: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeResponseFormatter();
                case 109: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesResponseFormatter();
                case 110: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeRequestFormatter();
                case 111: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeResponseFormatter();
                case 112: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.SetParamRequestFormatter();
                case 113: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersRequestFormatter();
                case 114: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersResponseFormatter();
                case 115: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeRequestFormatter();
                case 116: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeResponseFormatter();
                case 117: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsResponseFormatter();
                case 118: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeRequestFormatter();
                case 119: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeResponseFormatter();
                case 120: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.BatteryStateFormatter();
                case 121: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterestFormatter();
                case 122: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfoFormatter();
                case 123: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32Formatter();
                case 124: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImageFormatter();
                case 125: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.FluidPressureFormatter();
                case 126: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.IlluminanceFormatter();
                case 127: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.ImageFormatter();
                case 128: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.ImuFormatter();
                case 129: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.JointStateFormatter();
                case 130: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFormatter();
                case 131: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackFormatter();
                case 132: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackArrayFormatter();
                case 133: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEchoFormatter();
                case 134: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScanFormatter();
                case 135: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.MagneticFieldFormatter();
                case 136: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiDOFJointStateFormatter();
                case 137: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiEchoLaserScanFormatter();
                case 138: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatusFormatter();
                case 139: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatFixFormatter();
                case 140: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloudFormatter();
                case 141: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.PointFieldFormatter();
                case 142: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud2Formatter();
                case 143: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.RangeFormatter();
                case 144: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.RelativeHumidityFormatter();
                case 145: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.TemperatureFormatter();
                case 146: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.TimeReferenceFormatter();
                case 147: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoRequestFormatter();
                case 148: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoResponseFormatter();
                case 149: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangleFormatter();
                case 150: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Shape.MeshFormatter();
                case 151: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Shape.PlaneFormatter();
                case 152: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Shape.SolidPrimitiveFormatter();
                case 153: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.BoolFormatter();
                case 154: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.ByteFormatter();
                case 155: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimensionFormatter();
                case 156: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayoutFormatter();
                case 157: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.ByteMultiArrayFormatter();
                case 158: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.CharFormatter();
                case 159: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.ColorRGBAFormatter();
                case 160: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Float32Formatter();
                case 161: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArrayFormatter();
                case 162: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Float64Formatter();
                case 163: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Float64MultiArrayFormatter();
                case 164: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int16Formatter();
                case 165: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int16MultiArrayFormatter();
                case 166: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int32Formatter();
                case 167: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArrayFormatter();
                case 168: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int64Formatter();
                case 169: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int64MultiArrayFormatter();
                case 170: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int8Formatter();
                case 171: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.Int8MultiArrayFormatter();
                case 172: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.StringFormatter();
                case 173: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt16Formatter();
                case 174: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt16MultiArrayFormatter();
                case 175: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt32Formatter();
                case 176: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt32MultiArrayFormatter();
                case 177: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt64Formatter();
                case 178: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt64MultiArrayFormatter();
                case 179: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt8Formatter();
                case 180: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.UInt8MultiArrayFormatter();
                case 181: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolRequestFormatter();
                case 182: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolResponseFormatter();
                case 183: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std.TriggerResponseFormatter();
                case 184: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoalFormatter();
                case 185: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoalFormatter();
                case 186: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2ErrorFormatter();
                case 187: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResultFormatter();
                case 188: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResultFormatter();
                case 189: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedbackFormatter();
                case 190: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFormatter();
                case 191: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.TFMessageFormatter();
                case 192: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2.FrameGraphResponseFormatter();
                case 193: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPointFormatter();
                case 194: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryFormatter();
                case 195: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPointFormatter();
                case 196: return new Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryFormatter();
                default: return null;
            }
        }
    }
}

#pragma warning disable 168
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612

#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Std
{
    using System;
    using Utf8Json;


    public sealed class DurationFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public DurationFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("secs"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("nsecs"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("secs"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("nsecs"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt32(value.secs);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.nsecs);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __secs__ = default(uint);
            var __secs__b__ = false;
            var __nsecs__ = default(uint);
            var __nsecs__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __secs__ = reader.ReadUInt32();
                        __secs__b__ = true;
                        break;
                    case 1:
                        __nsecs__ = reader.ReadUInt32();
                        __nsecs__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration();
            if(__secs__b__) ____result.secs = __secs__;
            if(__nsecs__b__) ____result.nsecs = __nsecs__;

            return ____result;
        }
    }


    public sealed class TimeFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TimeFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("secs"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("nsecs"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("secs"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("nsecs"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Time value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt32(value.secs);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.nsecs);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Time Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __secs__ = default(uint);
            var __secs__b__ = false;
            var __nsecs__ = default(uint);
            var __nsecs__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __secs__ = reader.ReadUInt32();
                        __secs__b__ = true;
                        break;
                    case 1:
                        __nsecs__ = reader.ReadUInt32();
                        __nsecs__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Time();
            if(__secs__b__) ____result.secs = __secs__;
            if(__nsecs__b__) ____result.nsecs = __nsecs__;

            return ____result;
        }
    }


    public sealed class HeaderFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public HeaderFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("seq"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("stamp"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("frame_id"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("seq"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("stamp"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("frame_id"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Header value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt32(value.seq);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.stamp, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.frame_id);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Header Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __seq__ = default(uint);
            var __seq__b__ = false;
            var __stamp__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __stamp__b__ = false;
            var __frame_id__ = default(string);
            var __frame_id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __seq__ = reader.ReadUInt32();
                        __seq__b__ = true;
                        break;
                    case 1:
                        __stamp__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __stamp__b__ = true;
                        break;
                    case 2:
                        __frame_id__ = reader.ReadString();
                        __frame_id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Header();
            if(__seq__b__) ____result.seq = __seq__;
            if(__stamp__b__) ____result.stamp = __stamp__;
            if(__frame_id__b__) ____result.frame_id = __frame_id__;

            return ____result;
        }
    }


    public sealed class BoolFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Bool>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public BoolFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Bool value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Bool Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(bool);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadBoolean();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Bool();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class ByteFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Byte>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ByteFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Byte value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteSByte(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Byte Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(sbyte);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadSByte();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Byte();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class MultiArrayDimensionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MultiArrayDimensionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("label"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("size"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("stride"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("label"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("size"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("stride"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.label);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.size);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteUInt32(value.stride);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __label__ = default(string);
            var __label__b__ = false;
            var __size__ = default(uint);
            var __size__b__ = false;
            var __stride__ = default(uint);
            var __stride__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __label__ = reader.ReadString();
                        __label__b__ = true;
                        break;
                    case 1:
                        __size__ = reader.ReadUInt32();
                        __size__b__ = true;
                        break;
                    case 2:
                        __stride__ = reader.ReadUInt32();
                        __stride__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension();
            if(__label__b__) ____result.label = __label__;
            if(__size__b__) ____result.size = __size__;
            if(__stride__b__) ____result.stride = __stride__;

            return ____result;
        }
    }


    public sealed class MultiArrayLayoutFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MultiArrayLayoutFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("dim"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data_offset"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("dim"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data_offset"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension[]>().Serialize(ref writer, value.dim, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.data_offset);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __dim__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension[]);
            var __dim__b__ = false;
            var __data_offset__ = default(uint);
            var __data_offset__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __dim__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayDimension[]>().Deserialize(ref reader, formatterResolver);
                        __dim__b__ = true;
                        break;
                    case 1:
                        __data_offset__ = reader.ReadUInt32();
                        __data_offset__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout();
            if(__dim__b__) ____result.dim = __dim__;
            if(__data_offset__b__) ____result.data_offset = __data_offset__;

            return ____result;
        }
    }


    public sealed class ByteMultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.ByteMultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ByteMultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.ByteMultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<sbyte[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.ByteMultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(sbyte[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<sbyte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.ByteMultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class CharFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Char>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public CharFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Char value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteByte(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Char Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(byte);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadByte();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Char();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class ColorRGBAFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.ColorRGBA>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ColorRGBAFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("r"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("g"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("b"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("a"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("r"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("g"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("b"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("a"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.ColorRGBA value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteSingle(value.r);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.g);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.b);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteSingle(value.a);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.ColorRGBA Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __r__ = default(float);
            var __r__b__ = false;
            var __g__ = default(float);
            var __g__b__ = false;
            var __b__ = default(float);
            var __b__b__ = false;
            var __a__ = default(float);
            var __a__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __r__ = reader.ReadSingle();
                        __r__b__ = true;
                        break;
                    case 1:
                        __g__ = reader.ReadSingle();
                        __g__b__ = true;
                        break;
                    case 2:
                        __b__ = reader.ReadSingle();
                        __b__b__ = true;
                        break;
                    case 3:
                        __a__ = reader.ReadSingle();
                        __a__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.ColorRGBA();
            if(__r__b__) ____result.r = __r__;
            if(__g__b__) ____result.g = __g__;
            if(__b__b__) ____result.b = __b__;
            if(__a__b__) ____result.a = __a__;

            return ____result;
        }
    }


    public sealed class Float32Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Float32Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteSingle(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(float);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadSingle();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Float32MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Float32MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(float[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Float32MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Float64Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Float64Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteDouble(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(double);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadDouble();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Float64MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Float64MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(double[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Float64MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int16Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int16Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteInt16(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(short);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadInt16();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int16MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int16MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<short[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(short[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<short[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int16MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int32Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int32Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteInt32(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(int);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadInt32();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int32MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int32MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<int[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(int[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<int[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int64Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int64Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteInt64(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(long);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadInt64();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int64MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int64MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<long[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(long[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<long[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int64MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int8Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int8Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteSByte(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(sbyte);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadSByte();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class Int8MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Int8MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<sbyte[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(sbyte[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<sbyte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.Int8MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class StringFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.String>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public StringFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.String value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.String Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(string);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadString();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.String();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt16Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt16Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt16(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(ushort);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadUInt16();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt16MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt16MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<ushort[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(ushort[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<ushort[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt16MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt32Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt32Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt32(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(uint);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadUInt32();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt32MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt32MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<uint[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(uint[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<uint[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt32MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt64Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt64Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt64(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(ulong);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadUInt64();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt64MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt64MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<ulong[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(ulong[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<ulong[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt64MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt8Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt8Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteByte(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(byte);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadByte();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class UInt8MultiArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8MultiArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public UInt8MultiArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("layout"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("layout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8MultiArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Serialize(ref writer, value.layout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<byte[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8MultiArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __layout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout);
            var __layout__b__ = false;
            var __data__ = default(byte[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __layout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.MultiArrayLayout>().Deserialize(ref reader, formatterResolver);
                        __layout__b__ = true;
                        break;
                    case 1:
                        __data__ = formatterResolver.GetFormatterWithVerify<byte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.UInt8MultiArray();
            if(__layout__b__) ____result.layout = __layout__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class SetBoolRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetBoolRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.data);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __data__ = default(bool);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __data__ = reader.ReadBoolean();
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolRequest();
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class SetBoolResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetBoolResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("success"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("message"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("success"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("message"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.success);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.message);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __success__ = default(bool);
            var __success__b__ = false;
            var __message__ = default(string);
            var __message__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __success__ = reader.ReadBoolean();
                        __success__b__ = true;
                        break;
                    case 1:
                        __message__ = reader.ReadString();
                        __message__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.SetBoolResponse();
            if(__success__b__) ____result.success = __success__;
            if(__message__b__) ____result.message = __message__;

            return ____result;
        }
    }


    public sealed class TriggerResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TriggerResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("success"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("message"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("success"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("message"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.success);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.message);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __success__ = default(bool);
            var __success__b__ = false;
            var __message__ = default(string);
            var __message__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __success__ = reader.ReadBoolean();
                        __success__b__ = true;
                        break;
                    case 1:
                        __message__ = reader.ReadString();
                        __message__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Std.TriggerResponse();
            if(__success__b__) ____result.success = __success__;
            if(__message__b__) ____result.message = __message__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Actionlib
{
    using System;
    using Utf8Json;


    public sealed class GoalIDFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GoalIDFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("stamp"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("stamp"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.stamp, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.id);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __stamp__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __stamp__b__ = false;
            var __id__ = default(string);
            var __id__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __stamp__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __stamp__b__ = true;
                        break;
                    case 1:
                        __id__ = reader.ReadString();
                        __id__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID();
            if(__stamp__b__) ____result.stamp = __stamp__;
            if(__id__b__) ____result.id = __id__;

            return ____result;
        }
    }


    public sealed class GoalStatusFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GoalStatusFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal_id"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("text"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("goal_id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("text"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Serialize(ref writer, value.goal_id, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteByte(value.status);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.text);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __goal_id__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID);
            var __goal_id__b__ = false;
            var __status__ = default(byte);
            var __status__b__ = false;
            var __text__ = default(string);
            var __text__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __goal_id__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Deserialize(ref reader, formatterResolver);
                        __goal_id__b__ = true;
                        break;
                    case 1:
                        __status__ = reader.ReadByte();
                        __status__b__ = true;
                        break;
                    case 2:
                        __text__ = reader.ReadString();
                        __text__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus();
            if(__goal_id__b__) ____result.goal_id = __goal_id__;
            if(__status__b__) ____result.status = __status__;
            if(__text__b__) ____result.text = __text__;

            return ____result;
        }
    }


    public sealed class GoalStatusArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GoalStatusArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status_list"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status_list"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus[]>().Serialize(ref writer, value.status_list, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status_list__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus[]);
            var __status_list__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status_list__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus[]>().Deserialize(ref reader, formatterResolver);
                        __status_list__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatusArray();
            if(__header__b__) ____result.header = __header__;
            if(__status_list__b__) ____result.status_list = __status_list__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials
{
    using System;
    using Utf8Json;


    public sealed class FibonacciGoalFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciGoalFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("order"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("order"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteInt32(value.order);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __order__ = default(int);
            var __order__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __order__ = reader.ReadInt32();
                        __order__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal();
            if(__order__b__) ____result.order = __order__;

            return ____result;
        }
    }


    public sealed class FibonacciActionGoalFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciActionGoalFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal_id"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal_id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Serialize(ref writer, value.goal_id, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal>().Serialize(ref writer, value.goal, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __goal_id__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID);
            var __goal_id__b__ = false;
            var __goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal);
            var __goal__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __goal_id__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Deserialize(ref reader, formatterResolver);
                        __goal_id__b__ = true;
                        break;
                    case 2:
                        __goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciGoal>().Deserialize(ref reader, formatterResolver);
                        __goal__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal();
            if(__header__b__) ____result.header = __header__;
            if(__goal_id__b__) ____result.goal_id = __goal_id__;
            if(__goal__b__) ____result.goal = __goal__;

            return ____result;
        }
    }


    public sealed class FibonacciResultFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciResultFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("sequence"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("sequence"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<int[]>().Serialize(ref writer, value.sequence, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __sequence__ = default(int[]);
            var __sequence__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __sequence__ = formatterResolver.GetFormatterWithVerify<int[]>().Deserialize(ref reader, formatterResolver);
                        __sequence__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult();
            if(__sequence__b__) ____result.sequence = __sequence__;

            return ____result;
        }
    }


    public sealed class FibonacciActionResultFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciActionResultFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("result"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("result"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult>().Serialize(ref writer, value.result, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus);
            var __status__b__ = false;
            var __result__ = default(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult);
            var __result__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __result__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciResult>().Deserialize(ref reader, formatterResolver);
                        __result__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__result__b__) ____result.result = __result__;

            return ____result;
        }
    }


    public sealed class FibonacciFeedbackFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciFeedbackFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("sequence"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("sequence"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<int[]>().Serialize(ref writer, value.sequence, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __sequence__ = default(int[]);
            var __sequence__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __sequence__ = formatterResolver.GetFormatterWithVerify<int[]>().Deserialize(ref reader, formatterResolver);
                        __sequence__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback();
            if(__sequence__b__) ____result.sequence = __sequence__;

            return ____result;
        }
    }


    public sealed class FibonacciActionFeedbackFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciActionFeedbackFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("feedback"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("feedback"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback>().Serialize(ref writer, value.feedback, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus);
            var __status__b__ = false;
            var __feedback__ = default(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback);
            var __feedback__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __feedback__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciFeedback>().Deserialize(ref reader, formatterResolver);
                        __feedback__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__feedback__b__) ____result.feedback = __feedback__;

            return ____result;
        }
    }


    public sealed class FibonacciActionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciAction>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FibonacciActionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_goal"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_result"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_feedback"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("action_goal"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("action_result"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("action_feedback"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciAction value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal>().Serialize(ref writer, value.action_goal, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult>().Serialize(ref writer, value.action_result, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback>().Serialize(ref writer, value.action_feedback, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciAction Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __action_goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal);
            var __action_goal__b__ = false;
            var __action_result__ = default(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult);
            var __action_result__b__ = false;
            var __action_feedback__ = default(global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback);
            var __action_feedback__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __action_goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionGoal>().Deserialize(ref reader, formatterResolver);
                        __action_goal__b__ = true;
                        break;
                    case 1:
                        __action_result__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionResult>().Deserialize(ref reader, formatterResolver);
                        __action_result__b__ = true;
                        break;
                    case 2:
                        __action_feedback__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciActionFeedback>().Deserialize(ref reader, formatterResolver);
                        __action_feedback__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.ActionlibTutorials.FibonacciAction();
            if(__action_goal__b__) ____result.action_goal = __action_goal__;
            if(__action_result__b__) ____result.action_result = __action_result__;
            if(__action_feedback__b__) ____result.action_feedback = __action_feedback__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.FileServer
{
    using System;
    using Utf8Json;


    public sealed class GetBinaryFileRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetBinaryFileRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileRequest();
            if(__name__b__) ____result.name = __name__;

            return ____result;
        }
    }


    public sealed class GetBinaryFileResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetBinaryFileResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("value"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("value"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<byte[]>().Serialize(ref writer, value.value, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __value__ = default(byte[]);
            var __value__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __value__ = formatterResolver.GetFormatterWithVerify<byte[]>().Deserialize(ref reader, formatterResolver);
                        __value__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.FileServer.GetBinaryFileResponse();
            if(__value__b__) ____result.value = __value__;

            return ____result;
        }
    }


    public sealed class SaveBinaryFileRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SaveBinaryFileRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("value"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("value"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<byte[]>().Serialize(ref writer, value.value, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;
            var __value__ = default(byte[]);
            var __value__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    case 1:
                        __value__ = formatterResolver.GetFormatterWithVerify<byte[]>().Deserialize(ref reader, formatterResolver);
                        __value__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileRequest();
            if(__name__b__) ____result.name = __name__;
            if(__value__b__) ____result.value = __value__;

            return ____result;
        }
    }


    public sealed class SaveBinaryFileResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SaveBinaryFileResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.FileServer.SaveBinaryFileResponse();
            if(__name__b__) ____result.name = __name__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Geometry
{
    using System;
    using Utf8Json;


    public sealed class Vector3Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Vector3Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("x"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("y"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("z"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("x"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("y"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("z"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteDouble(value.x);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.y);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.z);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __x__ = default(double);
            var __x__b__ = false;
            var __y__ = default(double);
            var __y__b__ = false;
            var __z__ = default(double);
            var __z__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __x__ = reader.ReadDouble();
                        __x__b__ = true;
                        break;
                    case 1:
                        __y__ = reader.ReadDouble();
                        __y__b__ = true;
                        break;
                    case 2:
                        __z__ = reader.ReadDouble();
                        __z__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3();
            if(__x__b__) ____result.x = __x__;
            if(__y__b__) ____result.y = __y__;
            if(__z__b__) ____result.z = __z__;

            return ____result;
        }
    }


    public sealed class AccelFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public AccelFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("linear"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angular"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("linear"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angular"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.linear, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.angular, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __linear__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __linear__b__ = false;
            var __angular__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __angular__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __linear__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __linear__b__ = true;
                        break;
                    case 1:
                        __angular__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __angular__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel();
            if(__linear__b__) ____result.linear = __linear__;
            if(__angular__b__) ____result.angular = __angular__;

            return ____result;
        }
    }


    public sealed class AccelStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public AccelStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("accel"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("accel"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel>().Serialize(ref writer, value.accel, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __accel__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel);
            var __accel__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __accel__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel>().Deserialize(ref reader, formatterResolver);
                        __accel__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelStamped();
            if(__header__b__) ____result.header = __header__;
            if(__accel__b__) ____result.accel = __accel__;

            return ____result;
        }
    }


    public sealed class AccelWithCovarianceFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public AccelWithCovarianceFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("accel"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("covariance"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("accel"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("covariance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel>().Serialize(ref writer, value.accel, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.covariance, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __accel__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel);
            var __accel__b__ = false;
            var __covariance__ = default(double[]);
            var __covariance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __accel__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Accel>().Deserialize(ref reader, formatterResolver);
                        __accel__b__ = true;
                        break;
                    case 1:
                        __covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __covariance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance();
            if(__accel__b__) ____result.accel = __accel__;
            if(__covariance__b__) ____result.covariance = __covariance__;

            return ____result;
        }
    }


    public sealed class AccelWithCovarianceStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public AccelWithCovarianceStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("accel"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("accel"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance>().Serialize(ref writer, value.accel, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __accel__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance);
            var __accel__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __accel__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovariance>().Deserialize(ref reader, formatterResolver);
                        __accel__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.AccelWithCovarianceStamped();
            if(__header__b__) ____result.header = __header__;
            if(__accel__b__) ____result.accel = __accel__;

            return ____result;
        }
    }


    public sealed class InertiaFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public InertiaFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("m"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("com"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("ixx"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("ixy"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("ixz"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("iyy"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("iyz"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("izz"), 7},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("m"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("com"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("ixx"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("ixy"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("ixz"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("iyy"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("iyz"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("izz"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteDouble(value.m);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.com, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.ixx);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteDouble(value.ixy);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteDouble(value.ixz);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteDouble(value.iyy);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteDouble(value.iyz);
            writer.WriteRaw(this.____stringByteKeys[7]);
            writer.WriteDouble(value.izz);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __m__ = default(double);
            var __m__b__ = false;
            var __com__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __com__b__ = false;
            var __ixx__ = default(double);
            var __ixx__b__ = false;
            var __ixy__ = default(double);
            var __ixy__b__ = false;
            var __ixz__ = default(double);
            var __ixz__b__ = false;
            var __iyy__ = default(double);
            var __iyy__b__ = false;
            var __iyz__ = default(double);
            var __iyz__b__ = false;
            var __izz__ = default(double);
            var __izz__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __m__ = reader.ReadDouble();
                        __m__b__ = true;
                        break;
                    case 1:
                        __com__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __com__b__ = true;
                        break;
                    case 2:
                        __ixx__ = reader.ReadDouble();
                        __ixx__b__ = true;
                        break;
                    case 3:
                        __ixy__ = reader.ReadDouble();
                        __ixy__b__ = true;
                        break;
                    case 4:
                        __ixz__ = reader.ReadDouble();
                        __ixz__b__ = true;
                        break;
                    case 5:
                        __iyy__ = reader.ReadDouble();
                        __iyy__b__ = true;
                        break;
                    case 6:
                        __iyz__ = reader.ReadDouble();
                        __iyz__b__ = true;
                        break;
                    case 7:
                        __izz__ = reader.ReadDouble();
                        __izz__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia();
            if(__m__b__) ____result.m = __m__;
            if(__com__b__) ____result.com = __com__;
            if(__ixx__b__) ____result.ixx = __ixx__;
            if(__ixy__b__) ____result.ixy = __ixy__;
            if(__ixz__b__) ____result.ixz = __ixz__;
            if(__iyy__b__) ____result.iyy = __iyy__;
            if(__iyz__b__) ____result.iyz = __iyz__;
            if(__izz__b__) ____result.izz = __izz__;

            return ____result;
        }
    }


    public sealed class InertiaStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public InertiaStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("inertia"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("inertia"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia>().Serialize(ref writer, value.inertia, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __inertia__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia);
            var __inertia__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __inertia__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Inertia>().Deserialize(ref reader, formatterResolver);
                        __inertia__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.InertiaStamped();
            if(__header__b__) ____result.header = __header__;
            if(__inertia__b__) ____result.inertia = __inertia__;

            return ____result;
        }
    }


    public sealed class PointFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PointFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("x"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("y"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("z"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("x"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("y"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("z"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteDouble(value.x);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.y);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.z);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __x__ = default(double);
            var __x__b__ = false;
            var __y__ = default(double);
            var __y__b__ = false;
            var __z__ = default(double);
            var __z__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __x__ = reader.ReadDouble();
                        __x__b__ = true;
                        break;
                    case 1:
                        __y__ = reader.ReadDouble();
                        __y__b__ = true;
                        break;
                    case 2:
                        __z__ = reader.ReadDouble();
                        __z__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point();
            if(__x__b__) ____result.x = __x__;
            if(__y__b__) ____result.y = __y__;
            if(__z__b__) ____result.z = __z__;

            return ____result;
        }
    }


    public sealed class Point32Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Point32Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("x"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("y"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("z"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("x"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("y"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("z"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteSingle(value.x);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.y);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.z);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __x__ = default(float);
            var __x__b__ = false;
            var __y__ = default(float);
            var __y__b__ = false;
            var __z__ = default(float);
            var __z__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __x__ = reader.ReadSingle();
                        __x__b__ = true;
                        break;
                    case 1:
                        __y__ = reader.ReadSingle();
                        __y__b__ = true;
                        break;
                    case 2:
                        __z__ = reader.ReadSingle();
                        __z__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32();
            if(__x__b__) ____result.x = __x__;
            if(__y__b__) ____result.y = __y__;
            if(__z__b__) ____result.z = __z__;

            return ____result;
        }
    }


    public sealed class PointStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PointStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PointStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("point"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("point"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PointStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point>().Serialize(ref writer, value.point, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PointStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __point__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point);
            var __point__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __point__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point>().Deserialize(ref reader, formatterResolver);
                        __point__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PointStamped();
            if(__header__b__) ____result.header = __header__;
            if(__point__b__) ____result.point = __point__;

            return ____result;
        }
    }


    public sealed class PolygonFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PolygonFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("points"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("points"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]>().Serialize(ref writer, value.points, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __points__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]);
            var __points__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __points__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]>().Deserialize(ref reader, formatterResolver);
                        __points__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon();
            if(__points__b__) ____result.points = __points__;

            return ____result;
        }
    }


    public sealed class PolygonStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PolygonStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("polygon"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("polygon"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon>().Serialize(ref writer, value.polygon, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __polygon__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon);
            var __polygon__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __polygon__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Polygon>().Deserialize(ref reader, formatterResolver);
                        __polygon__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PolygonStamped();
            if(__header__b__) ____result.header = __header__;
            if(__polygon__b__) ____result.polygon = __polygon__;

            return ____result;
        }
    }


    public sealed class QuaternionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public QuaternionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("x"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("y"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("z"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("w"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("x"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("y"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("z"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("w"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteDouble(value.x);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.y);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.z);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteDouble(value.w);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __x__ = default(double);
            var __x__b__ = false;
            var __y__ = default(double);
            var __y__b__ = false;
            var __z__ = default(double);
            var __z__b__ = false;
            var __w__ = default(double);
            var __w__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __x__ = reader.ReadDouble();
                        __x__b__ = true;
                        break;
                    case 1:
                        __y__ = reader.ReadDouble();
                        __y__b__ = true;
                        break;
                    case 2:
                        __z__ = reader.ReadDouble();
                        __z__b__ = true;
                        break;
                    case 3:
                        __w__ = reader.ReadDouble();
                        __w__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion();
            if(__x__b__) ____result.x = __x__;
            if(__y__b__) ____result.y = __y__;
            if(__z__b__) ____result.z = __z__;
            if(__w__b__) ____result.w = __w__;

            return ____result;
        }
    }


    public sealed class PoseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PoseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("position"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("orientation"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("position"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("orientation"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point>().Serialize(ref writer, value.position, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Serialize(ref writer, value.orientation, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __position__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point);
            var __position__b__ = false;
            var __orientation__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion);
            var __orientation__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __position__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point>().Deserialize(ref reader, formatterResolver);
                        __position__b__ = true;
                        break;
                    case 1:
                        __orientation__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Deserialize(ref reader, formatterResolver);
                        __orientation__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose();
            if(__position__b__) ____result.position = __position__;
            if(__orientation__b__) ____result.orientation = __orientation__;

            return ____result;
        }
    }


    public sealed class Pose2DFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose2D>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Pose2DFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("x"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("y"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("theta"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("x"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("y"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("theta"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose2D value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteDouble(value.x);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.y);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.theta);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose2D Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __x__ = default(double);
            var __x__b__ = false;
            var __y__ = default(double);
            var __y__b__ = false;
            var __theta__ = default(double);
            var __theta__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __x__ = reader.ReadDouble();
                        __x__b__ = true;
                        break;
                    case 1:
                        __y__ = reader.ReadDouble();
                        __y__b__ = true;
                        break;
                    case 2:
                        __theta__ = reader.ReadDouble();
                        __theta__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose2D();
            if(__x__b__) ____result.x = __x__;
            if(__y__b__) ____result.y = __y__;
            if(__theta__b__) ____result.theta = __theta__;

            return ____result;
        }
    }


    public sealed class PoseArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PoseArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("poses"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("poses"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose[]>().Serialize(ref writer, value.poses, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __poses__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose[]);
            var __poses__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __poses__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose[]>().Deserialize(ref reader, formatterResolver);
                        __poses__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseArray();
            if(__header__b__) ____result.header = __header__;
            if(__poses__b__) ____result.poses = __poses__;

            return ____result;
        }
    }


    public sealed class PoseStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PoseStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("pose"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("pose"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>().Serialize(ref writer, value.pose, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __pose__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose);
            var __pose__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __pose__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>().Deserialize(ref reader, formatterResolver);
                        __pose__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped();
            if(__header__b__) ____result.header = __header__;
            if(__pose__b__) ____result.pose = __pose__;

            return ____result;
        }
    }


    public sealed class PoseWithCovarianceFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PoseWithCovarianceFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("pose"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("covariance"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("pose"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("covariance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>().Serialize(ref writer, value.pose, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.covariance, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __pose__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose);
            var __pose__b__ = false;
            var __covariance__ = default(double[]);
            var __covariance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __pose__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>().Deserialize(ref reader, formatterResolver);
                        __pose__b__ = true;
                        break;
                    case 1:
                        __covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __covariance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance();
            if(__pose__b__) ____result.pose = __pose__;
            if(__covariance__b__) ____result.covariance = __covariance__;

            return ____result;
        }
    }


    public sealed class PoseWithCovarianceStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PoseWithCovarianceStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("pose"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("pose"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance>().Serialize(ref writer, value.pose, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __pose__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance);
            var __pose__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __pose__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance>().Deserialize(ref reader, formatterResolver);
                        __pose__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped();
            if(__header__b__) ____result.header = __header__;
            if(__pose__b__) ____result.pose = __pose__;

            return ____result;
        }
    }


    public sealed class QuaternionStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public QuaternionStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("quaternion"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("quaternion"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Serialize(ref writer, value.quaternion, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __quaternion__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion);
            var __quaternion__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __quaternion__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Deserialize(ref reader, formatterResolver);
                        __quaternion__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.QuaternionStamped();
            if(__header__b__) ____result.header = __header__;
            if(__quaternion__b__) ____result.quaternion = __quaternion__;

            return ____result;
        }
    }


    public sealed class TransformFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TransformFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("translation"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("rotation"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("translation"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("rotation"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.translation, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Serialize(ref writer, value.rotation, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __translation__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __translation__b__ = false;
            var __rotation__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion);
            var __rotation__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __translation__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __translation__b__ = true;
                        break;
                    case 1:
                        __rotation__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Deserialize(ref reader, formatterResolver);
                        __rotation__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform();
            if(__translation__b__) ____result.translation = __translation__;
            if(__rotation__b__) ____result.rotation = __rotation__;

            return ____result;
        }
    }


    public sealed class TransformStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TransformStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("child_frame_id"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("transform"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("child_frame_id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("transform"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.child_frame_id);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform>().Serialize(ref writer, value.transform, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __child_frame_id__ = default(string);
            var __child_frame_id__b__ = false;
            var __transform__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform);
            var __transform__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __child_frame_id__ = reader.ReadString();
                        __child_frame_id__b__ = true;
                        break;
                    case 2:
                        __transform__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform>().Deserialize(ref reader, formatterResolver);
                        __transform__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped();
            if(__header__b__) ____result.header = __header__;
            if(__child_frame_id__b__) ____result.child_frame_id = __child_frame_id__;
            if(__transform__b__) ____result.transform = __transform__;

            return ____result;
        }
    }


    public sealed class TwistFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TwistFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("linear"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angular"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("linear"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angular"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.linear, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.angular, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __linear__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __linear__b__ = false;
            var __angular__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __angular__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __linear__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __linear__b__ = true;
                        break;
                    case 1:
                        __angular__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __angular__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist();
            if(__linear__b__) ____result.linear = __linear__;
            if(__angular__b__) ____result.angular = __angular__;

            return ____result;
        }
    }


    public sealed class TwistStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TwistStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("twist"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("twist"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>().Serialize(ref writer, value.twist, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __twist__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist);
            var __twist__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __twist__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>().Deserialize(ref reader, formatterResolver);
                        __twist__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistStamped();
            if(__header__b__) ____result.header = __header__;
            if(__twist__b__) ____result.twist = __twist__;

            return ____result;
        }
    }


    public sealed class TwistWithCovarianceFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TwistWithCovarianceFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("twist"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("covariance"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("twist"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("covariance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>().Serialize(ref writer, value.twist, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.covariance, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __twist__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist);
            var __twist__b__ = false;
            var __covariance__ = default(double[]);
            var __covariance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __twist__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>().Deserialize(ref reader, formatterResolver);
                        __twist__b__ = true;
                        break;
                    case 1:
                        __covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __covariance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance();
            if(__twist__b__) ____result.twist = __twist__;
            if(__covariance__b__) ____result.covariance = __covariance__;

            return ____result;
        }
    }


    public sealed class TwistWithCovarianceStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TwistWithCovarianceStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("twist"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("twist"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance>().Serialize(ref writer, value.twist, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __twist__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance);
            var __twist__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __twist__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance>().Deserialize(ref reader, formatterResolver);
                        __twist__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovarianceStamped();
            if(__header__b__) ____result.header = __header__;
            if(__twist__b__) ____result.twist = __twist__;

            return ____result;
        }
    }


    public sealed class Vector3StampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3Stamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public Vector3StampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("vector"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("vector"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3Stamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.vector, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3Stamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __vector__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __vector__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __vector__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __vector__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3Stamped();
            if(__header__b__) ____result.header = __header__;
            if(__vector__b__) ____result.vector = __vector__;

            return ____result;
        }
    }


    public sealed class WrenchFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public WrenchFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("force"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("torque"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("force"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("torque"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.force, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.torque, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __force__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __force__b__ = false;
            var __torque__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __torque__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __force__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __force__b__ = true;
                        break;
                    case 1:
                        __torque__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __torque__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench();
            if(__force__b__) ____result.force = __force__;
            if(__torque__b__) ____result.torque = __torque__;

            return ____result;
        }
    }


    public sealed class WrenchStampedFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchStamped>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public WrenchStampedFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("wrench"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("wrench"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchStamped value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench>().Serialize(ref writer, value.wrench, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchStamped Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __wrench__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench);
            var __wrench__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __wrench__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench>().Deserialize(ref reader, formatterResolver);
                        __wrench__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Geometry.WrenchStamped();
            if(__header__b__) ____result.header = __header__;
            if(__wrench__b__) ____result.wrench = __wrench__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Nav
{
    using System;
    using Utf8Json;


    public sealed class GetMapActionGoalFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetMapActionGoalFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal_id"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal_id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Serialize(ref writer, value.goal_id, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapGoal>().Serialize(ref writer, value.goal, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __goal_id__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID);
            var __goal_id__b__ = false;
            var __goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapGoal);
            var __goal__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __goal_id__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Deserialize(ref reader, formatterResolver);
                        __goal_id__b__ = true;
                        break;
                    case 2:
                        __goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapGoal>().Deserialize(ref reader, formatterResolver);
                        __goal__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal();
            if(__header__b__) ____result.header = __header__;
            if(__goal_id__b__) ____result.goal_id = __goal_id__;
            if(__goal__b__) ____result.goal = __goal__;

            return ____result;
        }
    }


    public sealed class MapMetaDataFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MapMetaDataFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("map_load_time"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("resolution"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("width"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("height"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("origin"), 4},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("map_load_time"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("resolution"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("width"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("height"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("origin"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.map_load_time, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.resolution);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteUInt32(value.width);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteUInt32(value.height);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>().Serialize(ref writer, value.origin, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __map_load_time__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __map_load_time__b__ = false;
            var __resolution__ = default(float);
            var __resolution__b__ = false;
            var __width__ = default(uint);
            var __width__b__ = false;
            var __height__ = default(uint);
            var __height__b__ = false;
            var __origin__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose);
            var __origin__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __map_load_time__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __map_load_time__b__ = true;
                        break;
                    case 1:
                        __resolution__ = reader.ReadSingle();
                        __resolution__b__ = true;
                        break;
                    case 2:
                        __width__ = reader.ReadUInt32();
                        __width__b__ = true;
                        break;
                    case 3:
                        __height__ = reader.ReadUInt32();
                        __height__b__ = true;
                        break;
                    case 4:
                        __origin__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>().Deserialize(ref reader, formatterResolver);
                        __origin__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData();
            if(__map_load_time__b__) ____result.map_load_time = __map_load_time__;
            if(__resolution__b__) ____result.resolution = __resolution__;
            if(__width__b__) ____result.width = __width__;
            if(__height__b__) ____result.height = __height__;
            if(__origin__b__) ____result.origin = __origin__;

            return ____result;
        }
    }


    public sealed class OccupancyGridFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public OccupancyGridFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("info"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("info"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData>().Serialize(ref writer, value.info, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<sbyte[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __info__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData);
            var __info__b__ = false;
            var __data__ = default(sbyte[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __info__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.MapMetaData>().Deserialize(ref reader, formatterResolver);
                        __info__b__ = true;
                        break;
                    case 2:
                        __data__ = formatterResolver.GetFormatterWithVerify<sbyte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid();
            if(__header__b__) ____result.header = __header__;
            if(__info__b__) ____result.info = __info__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class GetMapResultFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetMapResultFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("map"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("map"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>().Serialize(ref writer, value.map, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __map__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid);
            var __map__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __map__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>().Deserialize(ref reader, formatterResolver);
                        __map__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult();
            if(__map__b__) ____result.map = __map__;

            return ____result;
        }
    }


    public sealed class GetMapActionResultFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetMapActionResultFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("result"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("result"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult>().Serialize(ref writer, value.result, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus);
            var __status__b__ = false;
            var __result__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult);
            var __result__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __result__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResult>().Deserialize(ref reader, formatterResolver);
                        __result__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__result__b__) ____result.result = __result__;

            return ____result;
        }
    }


    public sealed class GetMapActionFeedbackFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetMapActionFeedbackFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("feedback"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("feedback"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapFeedback>().Serialize(ref writer, value.feedback, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus);
            var __status__b__ = false;
            var __feedback__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapFeedback);
            var __feedback__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __feedback__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapFeedback>().Deserialize(ref reader, formatterResolver);
                        __feedback__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__feedback__b__) ____result.feedback = __feedback__;

            return ____result;
        }
    }


    public sealed class GetMapActionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapAction>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetMapActionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_goal"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_result"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_feedback"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("action_goal"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("action_result"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("action_feedback"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapAction value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal>().Serialize(ref writer, value.action_goal, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult>().Serialize(ref writer, value.action_result, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback>().Serialize(ref writer, value.action_feedback, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapAction Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __action_goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal);
            var __action_goal__b__ = false;
            var __action_result__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult);
            var __action_result__b__ = false;
            var __action_feedback__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback);
            var __action_feedback__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __action_goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionGoal>().Deserialize(ref reader, formatterResolver);
                        __action_goal__b__ = true;
                        break;
                    case 1:
                        __action_result__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionResult>().Deserialize(ref reader, formatterResolver);
                        __action_result__b__ = true;
                        break;
                    case 2:
                        __action_feedback__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapActionFeedback>().Deserialize(ref reader, formatterResolver);
                        __action_feedback__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapAction();
            if(__action_goal__b__) ____result.action_goal = __action_goal__;
            if(__action_result__b__) ____result.action_result = __action_result__;
            if(__action_feedback__b__) ____result.action_feedback = __action_feedback__;

            return ____result;
        }
    }


    public sealed class GridCellsFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GridCells>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GridCellsFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("cell_width"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("cell_height"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("cells"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("cell_width"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("cell_height"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("cells"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GridCells value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.cell_width);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.cell_height);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]>().Serialize(ref writer, value.cells, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GridCells Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __cell_width__ = default(float);
            var __cell_width__b__ = false;
            var __cell_height__ = default(float);
            var __cell_height__b__ = false;
            var __cells__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]);
            var __cells__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __cell_width__ = reader.ReadSingle();
                        __cell_width__b__ = true;
                        break;
                    case 2:
                        __cell_height__ = reader.ReadSingle();
                        __cell_height__b__ = true;
                        break;
                    case 3:
                        __cells__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]>().Deserialize(ref reader, formatterResolver);
                        __cells__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GridCells();
            if(__header__b__) ____result.header = __header__;
            if(__cell_width__b__) ____result.cell_width = __cell_width__;
            if(__cell_height__b__) ____result.cell_height = __cell_height__;
            if(__cells__b__) ____result.cells = __cells__;

            return ____result;
        }
    }


    public sealed class OdometryFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.Odometry>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public OdometryFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("child_frame_id"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("pose"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("twist"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("child_frame_id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("pose"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("twist"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.Odometry value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.child_frame_id);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance>().Serialize(ref writer, value.pose, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance>().Serialize(ref writer, value.twist, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.Odometry Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __child_frame_id__ = default(string);
            var __child_frame_id__b__ = false;
            var __pose__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance);
            var __pose__b__ = false;
            var __twist__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance);
            var __twist__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __child_frame_id__ = reader.ReadString();
                        __child_frame_id__b__ = true;
                        break;
                    case 2:
                        __pose__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance>().Deserialize(ref reader, formatterResolver);
                        __pose__b__ = true;
                        break;
                    case 3:
                        __twist__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance>().Deserialize(ref reader, formatterResolver);
                        __twist__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.Odometry();
            if(__header__b__) ____result.header = __header__;
            if(__child_frame_id__b__) ____result.child_frame_id = __child_frame_id__;
            if(__pose__b__) ____result.pose = __pose__;
            if(__twist__b__) ____result.twist = __twist__;

            return ____result;
        }
    }


    public sealed class PathFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PathFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("poses"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("poses"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped[]>().Serialize(ref writer, value.poses, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __poses__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped[]);
            var __poses__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __poses__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped[]>().Deserialize(ref reader, formatterResolver);
                        __poses__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path();
            if(__header__b__) ____result.header = __header__;
            if(__poses__b__) ____result.poses = __poses__;

            return ____result;
        }
    }


    public sealed class GetMapResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetMapResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("map"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("map"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>().Serialize(ref writer, value.map, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __map__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid);
            var __map__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __map__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>().Deserialize(ref reader, formatterResolver);
                        __map__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetMapResponse();
            if(__map__b__) ____result.map = __map__;

            return ____result;
        }
    }


    public sealed class GetPlanRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetPlanRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("start"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("tolerance"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("start"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("tolerance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped>().Serialize(ref writer, value.start, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped>().Serialize(ref writer, value.goal, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.tolerance);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __start__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped);
            var __start__b__ = false;
            var __goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped);
            var __goal__b__ = false;
            var __tolerance__ = default(float);
            var __tolerance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __start__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped>().Deserialize(ref reader, formatterResolver);
                        __start__b__ = true;
                        break;
                    case 1:
                        __goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseStamped>().Deserialize(ref reader, formatterResolver);
                        __goal__b__ = true;
                        break;
                    case 2:
                        __tolerance__ = reader.ReadSingle();
                        __tolerance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanRequest();
            if(__start__b__) ____result.start = __start__;
            if(__goal__b__) ____result.goal = __goal__;
            if(__tolerance__b__) ____result.tolerance = __tolerance__;

            return ____result;
        }
    }


    public sealed class GetPlanResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetPlanResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("plan"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("plan"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path>().Serialize(ref writer, value.plan, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __plan__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path);
            var __plan__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __plan__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.Path>().Deserialize(ref reader, formatterResolver);
                        __plan__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.GetPlanResponse();
            if(__plan__b__) ____result.plan = __plan__;

            return ____result;
        }
    }


    public sealed class SetMapRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetMapRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("map"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("initial_pose"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("map"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("initial_pose"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>().Serialize(ref writer, value.map, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped>().Serialize(ref writer, value.initial_pose, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __map__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid);
            var __map__b__ = false;
            var __initial_pose__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped);
            var __initial_pose__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __map__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Nav.OccupancyGrid>().Deserialize(ref reader, formatterResolver);
                        __map__b__ = true;
                        break;
                    case 1:
                        __initial_pose__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovarianceStamped>().Deserialize(ref reader, formatterResolver);
                        __initial_pose__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapRequest();
            if(__map__b__) ____result.map = __map__;
            if(__initial_pose__b__) ____result.initial_pose = __initial_pose__;

            return ____result;
        }
    }


    public sealed class SetMapResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetMapResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("success"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("success"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.success);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __success__ = default(bool);
            var __success__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __success__ = reader.ReadBoolean();
                        __success__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Nav.SetMapResponse();
            if(__success__b__) ____result.success = __success__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Rosapi
{
    using System;
    using Utf8Json;


    public sealed class TypeDefFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TypeDefFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fieldnames"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fieldtypes"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fieldarraylen"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("examples"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("constnames"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("constvalues"), 6},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fieldnames"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fieldtypes"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fieldarraylen"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("examples"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("constnames"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("constvalues"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.fieldnames, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.fieldtypes, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<int[]>().Serialize(ref writer, value.fieldarraylen, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.examples, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[5]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.constnames, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[6]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.constvalues, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;
            var __fieldnames__ = default(string[]);
            var __fieldnames__b__ = false;
            var __fieldtypes__ = default(string[]);
            var __fieldtypes__b__ = false;
            var __fieldarraylen__ = default(int[]);
            var __fieldarraylen__b__ = false;
            var __examples__ = default(string[]);
            var __examples__b__ = false;
            var __constnames__ = default(string[]);
            var __constnames__b__ = false;
            var __constvalues__ = default(string[]);
            var __constvalues__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    case 1:
                        __fieldnames__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __fieldnames__b__ = true;
                        break;
                    case 2:
                        __fieldtypes__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __fieldtypes__b__ = true;
                        break;
                    case 3:
                        __fieldarraylen__ = formatterResolver.GetFormatterWithVerify<int[]>().Deserialize(ref reader, formatterResolver);
                        __fieldarraylen__b__ = true;
                        break;
                    case 4:
                        __examples__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __examples__b__ = true;
                        break;
                    case 5:
                        __constnames__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __constnames__b__ = true;
                        break;
                    case 6:
                        __constvalues__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __constvalues__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef();
            if(__type__b__) ____result.type = __type__;
            if(__fieldnames__b__) ____result.fieldnames = __fieldnames__;
            if(__fieldtypes__b__) ____result.fieldtypes = __fieldtypes__;
            if(__fieldarraylen__b__) ____result.fieldarraylen = __fieldarraylen__;
            if(__examples__b__) ____result.examples = __examples__;
            if(__constnames__b__) ____result.constnames = __constnames__;
            if(__constvalues__b__) ____result.constvalues = __constvalues__;

            return ____result;
        }
    }


    public sealed class DeleteParamRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.DeleteParamRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public DeleteParamRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.DeleteParamRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.DeleteParamRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.DeleteParamRequest();
            if(__name__b__) ____result.name = __name__;

            return ____result;
        }
    }


    public sealed class GetActionServersResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetActionServersResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetActionServersResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_servers"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("action_servers"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetActionServersResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.action_servers, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetActionServersResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __action_servers__ = default(string[]);
            var __action_servers__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __action_servers__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __action_servers__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetActionServersResponse();
            if(__action_servers__b__) ____result.action_servers = __action_servers__;

            return ____result;
        }
    }


    public sealed class GetParamNamesResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamNamesResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetParamNamesResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("names"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("names"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamNamesResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.names, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamNamesResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __names__ = default(string[]);
            var __names__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __names__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __names__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamNamesResponse();
            if(__names__b__) ____result.names = __names__;

            return ____result;
        }
    }


    public sealed class GetParamRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetParamRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("default"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("default"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value._default);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;
            var ___default__ = default(string);
            var ___default__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    case 1:
                        ___default__ = reader.ReadString();
                        ___default__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamRequest();
            if(__name__b__) ____result.name = __name__;
            if(___default__b__) ____result._default = ___default__;

            return ____result;
        }
    }


    public sealed class GetParamResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetParamResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("value"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("value"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.value);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __value__ = default(string);
            var __value__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __value__ = reader.ReadString();
                        __value__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetParamResponse();
            if(__value__b__) ____result.value = __value__;

            return ____result;
        }
    }


    public sealed class GetTimeResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetTimeResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public GetTimeResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("time"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("time"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetTimeResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.time, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetTimeResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __time__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __time__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __time__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __time__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.GetTimeResponse();
            if(__time__b__) ____result.time = __time__;

            return ____result;
        }
    }


    public sealed class HasParamRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public HasParamRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamRequest();
            if(__name__b__) ____result.name = __name__;

            return ____result;
        }
    }


    public sealed class HasParamResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public HasParamResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("exists"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("exists"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.exists);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __exists__ = default(bool);
            var __exists__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __exists__ = reader.ReadBoolean();
                        __exists__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.HasParamResponse();
            if(__exists__b__) ____result.exists = __exists__;

            return ____result;
        }
    }


    public sealed class MessageDetailsRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MessageDetailsRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsRequest();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }


    public sealed class MessageDetailsResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MessageDetailsResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("typedefs"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("typedefs"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]>().Serialize(ref writer, value.typedefs, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __typedefs__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]);
            var __typedefs__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __typedefs__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]>().Deserialize(ref reader, formatterResolver);
                        __typedefs__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.MessageDetailsResponse();
            if(__typedefs__b__) ____result.typedefs = __typedefs__;

            return ____result;
        }
    }


    public sealed class NodeDetailsRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public NodeDetailsRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("node"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("node"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.node);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __node__ = default(string);
            var __node__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __node__ = reader.ReadString();
                        __node__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsRequest();
            if(__node__b__) ____result.node = __node__;

            return ____result;
        }
    }


    public sealed class NodeDetailsResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public NodeDetailsResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("subscribing"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("publishing"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("services"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("subscribing"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("publishing"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("services"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.subscribing, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.publishing, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.services, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __subscribing__ = default(string[]);
            var __subscribing__b__ = false;
            var __publishing__ = default(string[]);
            var __publishing__b__ = false;
            var __services__ = default(string[]);
            var __services__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __subscribing__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __subscribing__b__ = true;
                        break;
                    case 1:
                        __publishing__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __publishing__b__ = true;
                        break;
                    case 2:
                        __services__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __services__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodeDetailsResponse();
            if(__subscribing__b__) ____result.subscribing = __subscribing__;
            if(__publishing__b__) ____result.publishing = __publishing__;
            if(__services__b__) ____result.services = __services__;

            return ____result;
        }
    }


    public sealed class NodesResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodesResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public NodesResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("nodes"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("nodes"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodesResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.nodes, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodesResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __nodes__ = default(string[]);
            var __nodes__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __nodes__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __nodes__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.NodesResponse();
            if(__nodes__b__) ____result.nodes = __nodes__;

            return ____result;
        }
    }


    public sealed class PublishersRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PublishersRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __topic__ = default(string);
            var __topic__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersRequest();
            if(__topic__b__) ____result.topic = __topic__;

            return ____result;
        }
    }


    public sealed class PublishersResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PublishersResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("publishers"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("publishers"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.publishers, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __publishers__ = default(string[]);
            var __publishers__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __publishers__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __publishers__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.PublishersResponse();
            if(__publishers__b__) ____result.publishers = __publishers__;

            return ____result;
        }
    }


    public sealed class SearchParamRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SearchParamRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamRequest();
            if(__name__b__) ____result.name = __name__;

            return ____result;
        }
    }


    public sealed class SearchParamResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SearchParamResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("global_name"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("global_name"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.global_name);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __global_name__ = default(string);
            var __global_name__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __global_name__ = reader.ReadString();
                        __global_name__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SearchParamResponse();
            if(__global_name__b__) ____result.global_name = __global_name__;

            return ____result;
        }
    }


    public sealed class ServiceHostRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceHostRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __service__ = default(string);
            var __service__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostRequest();
            if(__service__b__) ____result.service = __service__;

            return ____result;
        }
    }


    public sealed class ServiceHostResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceHostResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("host"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("host"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.host);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __host__ = default(string);
            var __host__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __host__ = reader.ReadString();
                        __host__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceHostResponse();
            if(__host__b__) ____result.host = __host__;

            return ____result;
        }
    }


    public sealed class ServiceNodeRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceNodeRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __service__ = default(string);
            var __service__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeRequest();
            if(__service__b__) ____result.service = __service__;

            return ____result;
        }
    }


    public sealed class ServiceNodeResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceNodeResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("node"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("node"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.node);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __node__ = default(string);
            var __node__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __node__ = reader.ReadString();
                        __node__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceNodeResponse();
            if(__node__b__) ____result.node = __node__;

            return ____result;
        }
    }


    public sealed class ServiceProvidersRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceProvidersRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __service__ = default(string);
            var __service__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersRequest();
            if(__service__b__) ____result.service = __service__;

            return ____result;
        }
    }


    public sealed class ServiceProvidersResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceProvidersResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("providers"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("providers"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.providers, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __providers__ = default(string[]);
            var __providers__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __providers__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __providers__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceProvidersResponse();
            if(__providers__b__) ____result.providers = __providers__;

            return ____result;
        }
    }


    public sealed class ServiceRequestDetailsRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceRequestDetailsRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsRequest();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }


    public sealed class ServiceRequestDetailsResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceRequestDetailsResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("typedefs"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("typedefs"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]>().Serialize(ref writer, value.typedefs, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __typedefs__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]);
            var __typedefs__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __typedefs__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]>().Deserialize(ref reader, formatterResolver);
                        __typedefs__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceRequestDetailsResponse();
            if(__typedefs__b__) ____result.typedefs = __typedefs__;

            return ____result;
        }
    }


    public sealed class ServiceResponseDetailsRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceResponseDetailsRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsRequest();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }


    public sealed class ServiceResponseDetailsResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceResponseDetailsResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("typedefs"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("typedefs"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]>().Serialize(ref writer, value.typedefs, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __typedefs__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]);
            var __typedefs__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __typedefs__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TypeDef[]>().Deserialize(ref reader, formatterResolver);
                        __typedefs__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceResponseDetailsResponse();
            if(__typedefs__b__) ____result.typedefs = __typedefs__;

            return ____result;
        }
    }


    public sealed class ServicesForTypeRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServicesForTypeRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeRequest();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }


    public sealed class ServicesForTypeResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServicesForTypeResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("services"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("services"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.services, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __services__ = default(string[]);
            var __services__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __services__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __services__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesForTypeResponse();
            if(__services__b__) ____result.services = __services__;

            return ____result;
        }
    }


    public sealed class ServicesResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServicesResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("services"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("services"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.services, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __services__ = default(string[]);
            var __services__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __services__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __services__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServicesResponse();
            if(__services__b__) ____result.services = __services__;

            return ____result;
        }
    }


    public sealed class ServiceTypeRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceTypeRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("service"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.service);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __service__ = default(string);
            var __service__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __service__ = reader.ReadString();
                        __service__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeRequest();
            if(__service__b__) ____result.service = __service__;

            return ____result;
        }
    }


    public sealed class ServiceTypeResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ServiceTypeResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.ServiceTypeResponse();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }


    public sealed class SetParamRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SetParamRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetParamRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("value"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("value"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SetParamRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.value);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SetParamRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;
            var __value__ = default(string);
            var __value__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    case 1:
                        __value__ = reader.ReadString();
                        __value__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SetParamRequest();
            if(__name__b__) ____result.name = __name__;
            if(__value__b__) ____result.value = __value__;

            return ____result;
        }
    }


    public sealed class SubscribersRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SubscribersRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __topic__ = default(string);
            var __topic__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersRequest();
            if(__topic__b__) ____result.topic = __topic__;

            return ____result;
        }
    }


    public sealed class SubscribersResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SubscribersResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("subscribers"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("subscribers"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.subscribers, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __subscribers__ = default(string[]);
            var __subscribers__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __subscribers__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __subscribers__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.SubscribersResponse();
            if(__subscribers__b__) ____result.subscribers = __subscribers__;

            return ____result;
        }
    }


    public sealed class TopicsForTypeRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TopicsForTypeRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeRequest();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }


    public sealed class TopicsForTypeResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TopicsForTypeResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topics"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topics"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.topics, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __topics__ = default(string[]);
            var __topics__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topics__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __topics__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsForTypeResponse();
            if(__topics__b__) ____result.topics = __topics__;

            return ____result;
        }
    }


    public sealed class TopicsResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TopicsResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topics"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("types"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topics"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("types"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.topics, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.types, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __topics__ = default(string[]);
            var __topics__b__ = false;
            var __types__ = default(string[]);
            var __types__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topics__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __topics__b__ = true;
                        break;
                    case 1:
                        __types__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __types__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicsResponse();
            if(__topics__b__) ____result.topics = __topics__;
            if(__types__b__) ____result.types = __types__;

            return ____result;
        }
    }


    public sealed class TopicTypeRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TopicTypeRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("topic"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("topic"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.topic);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __topic__ = default(string);
            var __topic__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __topic__ = reader.ReadString();
                        __topic__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeRequest();
            if(__topic__b__) ____result.topic = __topic__;

            return ____result;
        }
    }


    public sealed class TopicTypeResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TopicTypeResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(string);
            var __type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadString();
                        __type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Rosapi.TopicTypeResponse();
            if(__type__b__) ____result.type = __type__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Sensor
{
    using System;
    using Utf8Json;


    public sealed class BatteryStateFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.BatteryState>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public BatteryStateFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("voltage"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("current"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("charge"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("capacity"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("design_capacity"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("percentage"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("power_supply_status"), 7},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("power_supply_health"), 8},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("power_supply_technology"), 9},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("present"), 10},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("cell_voltage"), 11},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("location"), 12},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("serial_number"), 13},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("voltage"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("current"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("charge"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("capacity"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("design_capacity"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("percentage"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("power_supply_status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("power_supply_health"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("power_supply_technology"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("present"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("cell_voltage"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("location"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("serial_number"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.BatteryState value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.voltage);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.current);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteSingle(value.charge);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteSingle(value.capacity);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteSingle(value.design_capacity);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteSingle(value.percentage);
            writer.WriteRaw(this.____stringByteKeys[7]);
            writer.WriteByte(value.power_supply_status);
            writer.WriteRaw(this.____stringByteKeys[8]);
            writer.WriteByte(value.power_supply_health);
            writer.WriteRaw(this.____stringByteKeys[9]);
            writer.WriteByte(value.power_supply_technology);
            writer.WriteRaw(this.____stringByteKeys[10]);
            writer.WriteBoolean(value.present);
            writer.WriteRaw(this.____stringByteKeys[11]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.cell_voltage, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[12]);
            writer.WriteString(value.location);
            writer.WriteRaw(this.____stringByteKeys[13]);
            writer.WriteString(value.serial_number);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.BatteryState Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __voltage__ = default(float);
            var __voltage__b__ = false;
            var __current__ = default(float);
            var __current__b__ = false;
            var __charge__ = default(float);
            var __charge__b__ = false;
            var __capacity__ = default(float);
            var __capacity__b__ = false;
            var __design_capacity__ = default(float);
            var __design_capacity__b__ = false;
            var __percentage__ = default(float);
            var __percentage__b__ = false;
            var __power_supply_status__ = default(byte);
            var __power_supply_status__b__ = false;
            var __power_supply_health__ = default(byte);
            var __power_supply_health__b__ = false;
            var __power_supply_technology__ = default(byte);
            var __power_supply_technology__b__ = false;
            var __present__ = default(bool);
            var __present__b__ = false;
            var __cell_voltage__ = default(float[]);
            var __cell_voltage__b__ = false;
            var __location__ = default(string);
            var __location__b__ = false;
            var __serial_number__ = default(string);
            var __serial_number__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __voltage__ = reader.ReadSingle();
                        __voltage__b__ = true;
                        break;
                    case 2:
                        __current__ = reader.ReadSingle();
                        __current__b__ = true;
                        break;
                    case 3:
                        __charge__ = reader.ReadSingle();
                        __charge__b__ = true;
                        break;
                    case 4:
                        __capacity__ = reader.ReadSingle();
                        __capacity__b__ = true;
                        break;
                    case 5:
                        __design_capacity__ = reader.ReadSingle();
                        __design_capacity__b__ = true;
                        break;
                    case 6:
                        __percentage__ = reader.ReadSingle();
                        __percentage__b__ = true;
                        break;
                    case 7:
                        __power_supply_status__ = reader.ReadByte();
                        __power_supply_status__b__ = true;
                        break;
                    case 8:
                        __power_supply_health__ = reader.ReadByte();
                        __power_supply_health__b__ = true;
                        break;
                    case 9:
                        __power_supply_technology__ = reader.ReadByte();
                        __power_supply_technology__b__ = true;
                        break;
                    case 10:
                        __present__ = reader.ReadBoolean();
                        __present__b__ = true;
                        break;
                    case 11:
                        __cell_voltage__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __cell_voltage__b__ = true;
                        break;
                    case 12:
                        __location__ = reader.ReadString();
                        __location__b__ = true;
                        break;
                    case 13:
                        __serial_number__ = reader.ReadString();
                        __serial_number__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.BatteryState();
            if(__header__b__) ____result.header = __header__;
            if(__voltage__b__) ____result.voltage = __voltage__;
            if(__current__b__) ____result.current = __current__;
            if(__charge__b__) ____result.charge = __charge__;
            if(__capacity__b__) ____result.capacity = __capacity__;
            if(__design_capacity__b__) ____result.design_capacity = __design_capacity__;
            if(__percentage__b__) ____result.percentage = __percentage__;
            if(__power_supply_status__b__) ____result.power_supply_status = __power_supply_status__;
            if(__power_supply_health__b__) ____result.power_supply_health = __power_supply_health__;
            if(__power_supply_technology__b__) ____result.power_supply_technology = __power_supply_technology__;
            if(__present__b__) ____result.present = __present__;
            if(__cell_voltage__b__) ____result.cell_voltage = __cell_voltage__;
            if(__location__b__) ____result.location = __location__;
            if(__serial_number__b__) ____result.serial_number = __serial_number__;

            return ____result;
        }
    }


    public sealed class RegionOfInterestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public RegionOfInterestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("x_offset"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("y_offset"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("height"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("width"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("do_rectify"), 4},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("x_offset"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("y_offset"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("height"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("width"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("do_rectify"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteUInt32(value.x_offset);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.y_offset);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteUInt32(value.height);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteUInt32(value.width);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteBoolean(value.do_rectify);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __x_offset__ = default(uint);
            var __x_offset__b__ = false;
            var __y_offset__ = default(uint);
            var __y_offset__b__ = false;
            var __height__ = default(uint);
            var __height__b__ = false;
            var __width__ = default(uint);
            var __width__b__ = false;
            var __do_rectify__ = default(bool);
            var __do_rectify__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __x_offset__ = reader.ReadUInt32();
                        __x_offset__b__ = true;
                        break;
                    case 1:
                        __y_offset__ = reader.ReadUInt32();
                        __y_offset__b__ = true;
                        break;
                    case 2:
                        __height__ = reader.ReadUInt32();
                        __height__b__ = true;
                        break;
                    case 3:
                        __width__ = reader.ReadUInt32();
                        __width__b__ = true;
                        break;
                    case 4:
                        __do_rectify__ = reader.ReadBoolean();
                        __do_rectify__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest();
            if(__x_offset__b__) ____result.x_offset = __x_offset__;
            if(__y_offset__b__) ____result.y_offset = __y_offset__;
            if(__height__b__) ____result.height = __height__;
            if(__width__b__) ____result.width = __width__;
            if(__do_rectify__b__) ____result.do_rectify = __do_rectify__;

            return ____result;
        }
    }


    public sealed class CameraInfoFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public CameraInfoFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("height"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("width"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("distortion_model"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("D"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("K"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("R"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("P"), 7},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("binning_x"), 8},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("binning_y"), 9},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("roi"), 10},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("height"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("width"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("distortion_model"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("D"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("K"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("R"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("P"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("binning_x"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("binning_y"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("roi"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.height);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteUInt32(value.width);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.distortion_model);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.D, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[5]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.K, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[6]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.R, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[7]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.P, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[8]);
            writer.WriteUInt32(value.binning_x);
            writer.WriteRaw(this.____stringByteKeys[9]);
            writer.WriteUInt32(value.binning_y);
            writer.WriteRaw(this.____stringByteKeys[10]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest>().Serialize(ref writer, value.roi, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __height__ = default(uint);
            var __height__b__ = false;
            var __width__ = default(uint);
            var __width__b__ = false;
            var __distortion_model__ = default(string);
            var __distortion_model__b__ = false;
            var __D__ = default(double[]);
            var __D__b__ = false;
            var __K__ = default(double[]);
            var __K__b__ = false;
            var __R__ = default(double[]);
            var __R__b__ = false;
            var __P__ = default(double[]);
            var __P__b__ = false;
            var __binning_x__ = default(uint);
            var __binning_x__b__ = false;
            var __binning_y__ = default(uint);
            var __binning_y__b__ = false;
            var __roi__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest);
            var __roi__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __height__ = reader.ReadUInt32();
                        __height__b__ = true;
                        break;
                    case 2:
                        __width__ = reader.ReadUInt32();
                        __width__b__ = true;
                        break;
                    case 3:
                        __distortion_model__ = reader.ReadString();
                        __distortion_model__b__ = true;
                        break;
                    case 4:
                        __D__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __D__b__ = true;
                        break;
                    case 5:
                        __K__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __K__b__ = true;
                        break;
                    case 6:
                        __R__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __R__b__ = true;
                        break;
                    case 7:
                        __P__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __P__b__ = true;
                        break;
                    case 8:
                        __binning_x__ = reader.ReadUInt32();
                        __binning_x__b__ = true;
                        break;
                    case 9:
                        __binning_y__ = reader.ReadUInt32();
                        __binning_y__b__ = true;
                        break;
                    case 10:
                        __roi__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RegionOfInterest>().Deserialize(ref reader, formatterResolver);
                        __roi__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo();
            if(__header__b__) ____result.header = __header__;
            if(__height__b__) ____result.height = __height__;
            if(__width__b__) ____result.width = __width__;
            if(__distortion_model__b__) ____result.distortion_model = __distortion_model__;
            if(__D__b__) ____result.D = __D__;
            if(__K__b__) ____result.K = __K__;
            if(__R__b__) ____result.R = __R__;
            if(__P__b__) ____result.P = __P__;
            if(__binning_x__b__) ____result.binning_x = __binning_x__;
            if(__binning_y__b__) ____result.binning_y = __binning_y__;
            if(__roi__b__) ____result.roi = __roi__;

            return ____result;
        }
    }


    public sealed class ChannelFloat32Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ChannelFloat32Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("values"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("values"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.values, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;
            var __values__ = default(float[]);
            var __values__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    case 1:
                        __values__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __values__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32();
            if(__name__b__) ____result.name = __name__;
            if(__values__b__) ____result.values = __values__;

            return ____result;
        }
    }


    public sealed class CompressedImageFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public CompressedImageFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("format"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("format"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.format);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<byte[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __format__ = default(string);
            var __format__b__ = false;
            var __data__ = default(byte[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __format__ = reader.ReadString();
                        __format__b__ = true;
                        break;
                    case 2:
                        __data__ = formatterResolver.GetFormatterWithVerify<byte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage();
            if(__header__b__) ____result.header = __header__;
            if(__format__b__) ____result.format = __format__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class FluidPressureFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.FluidPressure>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FluidPressureFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fluid_pressure"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("variance"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fluid_pressure"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("variance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.FluidPressure value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.fluid_pressure);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.variance);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.FluidPressure Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __fluid_pressure__ = default(double);
            var __fluid_pressure__b__ = false;
            var __variance__ = default(double);
            var __variance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __fluid_pressure__ = reader.ReadDouble();
                        __fluid_pressure__b__ = true;
                        break;
                    case 2:
                        __variance__ = reader.ReadDouble();
                        __variance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.FluidPressure();
            if(__header__b__) ____result.header = __header__;
            if(__fluid_pressure__b__) ____result.fluid_pressure = __fluid_pressure__;
            if(__variance__b__) ____result.variance = __variance__;

            return ____result;
        }
    }


    public sealed class IlluminanceFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Illuminance>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public IlluminanceFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("illuminance"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("variance"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("illuminance"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("variance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Illuminance value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.illuminance);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.variance);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Illuminance Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __illuminance__ = default(double);
            var __illuminance__b__ = false;
            var __variance__ = default(double);
            var __variance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __illuminance__ = reader.ReadDouble();
                        __illuminance__b__ = true;
                        break;
                    case 2:
                        __variance__ = reader.ReadDouble();
                        __variance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Illuminance();
            if(__header__b__) ____result.header = __header__;
            if(__illuminance__b__) ____result.illuminance = __illuminance__;
            if(__variance__b__) ____result.variance = __variance__;

            return ____result;
        }
    }


    public sealed class ImageFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Image>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ImageFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("height"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("width"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("encoding"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("is_bigendian"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("step"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 6},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("height"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("width"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("encoding"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("is_bigendian"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("step"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Image value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.height);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteUInt32(value.width);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteString(value.encoding);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteByte(value.is_bigendian);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteUInt32(value.step);
            writer.WriteRaw(this.____stringByteKeys[6]);
            formatterResolver.GetFormatterWithVerify<byte[]>().Serialize(ref writer, value.data, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Image Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __height__ = default(uint);
            var __height__b__ = false;
            var __width__ = default(uint);
            var __width__b__ = false;
            var __encoding__ = default(string);
            var __encoding__b__ = false;
            var __is_bigendian__ = default(byte);
            var __is_bigendian__b__ = false;
            var __step__ = default(uint);
            var __step__b__ = false;
            var __data__ = default(byte[]);
            var __data__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __height__ = reader.ReadUInt32();
                        __height__b__ = true;
                        break;
                    case 2:
                        __width__ = reader.ReadUInt32();
                        __width__b__ = true;
                        break;
                    case 3:
                        __encoding__ = reader.ReadString();
                        __encoding__b__ = true;
                        break;
                    case 4:
                        __is_bigendian__ = reader.ReadByte();
                        __is_bigendian__b__ = true;
                        break;
                    case 5:
                        __step__ = reader.ReadUInt32();
                        __step__b__ = true;
                        break;
                    case 6:
                        __data__ = formatterResolver.GetFormatterWithVerify<byte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Image();
            if(__header__b__) ____result.header = __header__;
            if(__height__b__) ____result.height = __height__;
            if(__width__b__) ____result.width = __width__;
            if(__encoding__b__) ____result.encoding = __encoding__;
            if(__is_bigendian__b__) ____result.is_bigendian = __is_bigendian__;
            if(__step__b__) ____result.step = __step__;
            if(__data__b__) ____result.data = __data__;

            return ____result;
        }
    }


    public sealed class ImuFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Imu>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public ImuFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("orientation"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("orientation_covariance"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angular_velocity"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angular_velocity_covariance"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("linear_acceleration"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("linear_acceleration_covariance"), 6},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("orientation"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("orientation_covariance"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angular_velocity"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angular_velocity_covariance"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("linear_acceleration"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("linear_acceleration_covariance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Imu value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Serialize(ref writer, value.orientation, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.orientation_covariance, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.angular_velocity, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.angular_velocity_covariance, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[5]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.linear_acceleration, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[6]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.linear_acceleration_covariance, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Imu Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __orientation__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion);
            var __orientation__b__ = false;
            var __orientation_covariance__ = default(double[]);
            var __orientation_covariance__b__ = false;
            var __angular_velocity__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __angular_velocity__b__ = false;
            var __angular_velocity_covariance__ = default(double[]);
            var __angular_velocity_covariance__b__ = false;
            var __linear_acceleration__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __linear_acceleration__b__ = false;
            var __linear_acceleration_covariance__ = default(double[]);
            var __linear_acceleration_covariance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __orientation__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion>().Deserialize(ref reader, formatterResolver);
                        __orientation__b__ = true;
                        break;
                    case 2:
                        __orientation_covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __orientation_covariance__b__ = true;
                        break;
                    case 3:
                        __angular_velocity__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __angular_velocity__b__ = true;
                        break;
                    case 4:
                        __angular_velocity_covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __angular_velocity_covariance__b__ = true;
                        break;
                    case 5:
                        __linear_acceleration__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __linear_acceleration__b__ = true;
                        break;
                    case 6:
                        __linear_acceleration_covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __linear_acceleration_covariance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Imu();
            if(__header__b__) ____result.header = __header__;
            if(__orientation__b__) ____result.orientation = __orientation__;
            if(__orientation_covariance__b__) ____result.orientation_covariance = __orientation_covariance__;
            if(__angular_velocity__b__) ____result.angular_velocity = __angular_velocity__;
            if(__angular_velocity_covariance__b__) ____result.angular_velocity_covariance = __angular_velocity_covariance__;
            if(__linear_acceleration__b__) ____result.linear_acceleration = __linear_acceleration__;
            if(__linear_acceleration_covariance__b__) ____result.linear_acceleration_covariance = __linear_acceleration_covariance__;

            return ____result;
        }
    }


    public sealed class JointStateFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public JointStateFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("position"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("velocity"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("effort"), 4},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("name"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("position"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("velocity"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("effort"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.name, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.position, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.velocity, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.effort, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __name__ = default(string[]);
            var __name__b__ = false;
            var __position__ = default(double[]);
            var __position__b__ = false;
            var __velocity__ = default(double[]);
            var __velocity__b__ = false;
            var __effort__ = default(double[]);
            var __effort__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __name__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __name__b__ = true;
                        break;
                    case 2:
                        __position__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __position__b__ = true;
                        break;
                    case 3:
                        __velocity__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __velocity__b__ = true;
                        break;
                    case 4:
                        __effort__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __effort__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JointState();
            if(__header__b__) ____result.header = __header__;
            if(__name__b__) ____result.name = __name__;
            if(__position__b__) ____result.position = __position__;
            if(__velocity__b__) ____result.velocity = __velocity__;
            if(__effort__b__) ____result.effort = __effort__;

            return ____result;
        }
    }


    public sealed class JoyFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Joy>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public JoyFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("axes"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("buttons"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("axes"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("buttons"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Joy value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.axes, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<int[]>().Serialize(ref writer, value.buttons, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Joy Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __axes__ = default(float[]);
            var __axes__b__ = false;
            var __buttons__ = default(int[]);
            var __buttons__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __axes__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __axes__b__ = true;
                        break;
                    case 2:
                        __buttons__ = formatterResolver.GetFormatterWithVerify<int[]>().Deserialize(ref reader, formatterResolver);
                        __buttons__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Joy();
            if(__header__b__) ____result.header = __header__;
            if(__axes__b__) ____result.axes = __axes__;
            if(__buttons__b__) ____result.buttons = __buttons__;

            return ____result;
        }
    }


    public sealed class JoyFeedbackFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public JoyFeedbackFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("id"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("intensity"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("intensity"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteByte(value.type);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteByte(value.id);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.intensity);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(byte);
            var __type__b__ = false;
            var __id__ = default(byte);
            var __id__b__ = false;
            var __intensity__ = default(float);
            var __intensity__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadByte();
                        __type__b__ = true;
                        break;
                    case 1:
                        __id__ = reader.ReadByte();
                        __id__b__ = true;
                        break;
                    case 2:
                        __intensity__ = reader.ReadSingle();
                        __intensity__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback();
            if(__type__b__) ____result.type = __type__;
            if(__id__b__) ____result.id = __id__;
            if(__intensity__b__) ____result.intensity = __intensity__;

            return ____result;
        }
    }


    public sealed class JoyFeedbackArrayFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackArray>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public JoyFeedbackArrayFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("array"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("array"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackArray value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback[]>().Serialize(ref writer, value.array, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackArray Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __array__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback[]);
            var __array__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __array__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedback[]>().Deserialize(ref reader, formatterResolver);
                        __array__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.JoyFeedbackArray();
            if(__array__b__) ____result.array = __array__;

            return ____result;
        }
    }


    public sealed class LaserEchoFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LaserEchoFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("echoes"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("echoes"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.echoes, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __echoes__ = default(float[]);
            var __echoes__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __echoes__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __echoes__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho();
            if(__echoes__b__) ____result.echoes = __echoes__;

            return ____result;
        }
    }


    public sealed class LaserScanFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScan>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LaserScanFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angle_min"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angle_max"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angle_increment"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("time_increment"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("scan_time"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("range_min"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("range_max"), 7},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("ranges"), 8},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("intensities"), 9},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angle_min"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angle_max"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angle_increment"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("time_increment"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("scan_time"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("range_min"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("range_max"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("ranges"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("intensities"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScan value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.angle_min);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.angle_max);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteSingle(value.angle_increment);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteSingle(value.time_increment);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteSingle(value.scan_time);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteSingle(value.range_min);
            writer.WriteRaw(this.____stringByteKeys[7]);
            writer.WriteSingle(value.range_max);
            writer.WriteRaw(this.____stringByteKeys[8]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.ranges, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[9]);
            formatterResolver.GetFormatterWithVerify<float[]>().Serialize(ref writer, value.intensities, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScan Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __angle_min__ = default(float);
            var __angle_min__b__ = false;
            var __angle_max__ = default(float);
            var __angle_max__b__ = false;
            var __angle_increment__ = default(float);
            var __angle_increment__b__ = false;
            var __time_increment__ = default(float);
            var __time_increment__b__ = false;
            var __scan_time__ = default(float);
            var __scan_time__b__ = false;
            var __range_min__ = default(float);
            var __range_min__b__ = false;
            var __range_max__ = default(float);
            var __range_max__b__ = false;
            var __ranges__ = default(float[]);
            var __ranges__b__ = false;
            var __intensities__ = default(float[]);
            var __intensities__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __angle_min__ = reader.ReadSingle();
                        __angle_min__b__ = true;
                        break;
                    case 2:
                        __angle_max__ = reader.ReadSingle();
                        __angle_max__b__ = true;
                        break;
                    case 3:
                        __angle_increment__ = reader.ReadSingle();
                        __angle_increment__b__ = true;
                        break;
                    case 4:
                        __time_increment__ = reader.ReadSingle();
                        __time_increment__b__ = true;
                        break;
                    case 5:
                        __scan_time__ = reader.ReadSingle();
                        __scan_time__b__ = true;
                        break;
                    case 6:
                        __range_min__ = reader.ReadSingle();
                        __range_min__b__ = true;
                        break;
                    case 7:
                        __range_max__ = reader.ReadSingle();
                        __range_max__b__ = true;
                        break;
                    case 8:
                        __ranges__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __ranges__b__ = true;
                        break;
                    case 9:
                        __intensities__ = formatterResolver.GetFormatterWithVerify<float[]>().Deserialize(ref reader, formatterResolver);
                        __intensities__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserScan();
            if(__header__b__) ____result.header = __header__;
            if(__angle_min__b__) ____result.angle_min = __angle_min__;
            if(__angle_max__b__) ____result.angle_max = __angle_max__;
            if(__angle_increment__b__) ____result.angle_increment = __angle_increment__;
            if(__time_increment__b__) ____result.time_increment = __time_increment__;
            if(__scan_time__b__) ____result.scan_time = __scan_time__;
            if(__range_min__b__) ____result.range_min = __range_min__;
            if(__range_max__b__) ____result.range_max = __range_max__;
            if(__ranges__b__) ____result.ranges = __ranges__;
            if(__intensities__b__) ____result.intensities = __intensities__;

            return ____result;
        }
    }


    public sealed class MagneticFieldFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MagneticField>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MagneticFieldFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("magnetic_field"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("magnetic_field_covariance"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("magnetic_field"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("magnetic_field_covariance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MagneticField value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Serialize(ref writer, value.magnetic_field, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.magnetic_field_covariance, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MagneticField Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __magnetic_field__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3);
            var __magnetic_field__b__ = false;
            var __magnetic_field_covariance__ = default(double[]);
            var __magnetic_field_covariance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __magnetic_field__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3>().Deserialize(ref reader, formatterResolver);
                        __magnetic_field__b__ = true;
                        break;
                    case 2:
                        __magnetic_field_covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __magnetic_field_covariance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MagneticField();
            if(__header__b__) ____result.header = __header__;
            if(__magnetic_field__b__) ____result.magnetic_field = __magnetic_field__;
            if(__magnetic_field_covariance__b__) ____result.magnetic_field_covariance = __magnetic_field_covariance__;

            return ____result;
        }
    }


    public sealed class MultiDOFJointStateFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiDOFJointState>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MultiDOFJointStateFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("joint_names"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("transforms"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("twist"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("wrench"), 4},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("joint_names"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("transforms"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("twist"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("wrench"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiDOFJointState value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.joint_names, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]>().Serialize(ref writer, value.transforms, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]>().Serialize(ref writer, value.twist, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench[]>().Serialize(ref writer, value.wrench, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiDOFJointState Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __joint_names__ = default(string[]);
            var __joint_names__b__ = false;
            var __transforms__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]);
            var __transforms__b__ = false;
            var __twist__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]);
            var __twist__b__ = false;
            var __wrench__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench[]);
            var __wrench__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __joint_names__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __joint_names__b__ = true;
                        break;
                    case 2:
                        __transforms__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]>().Deserialize(ref reader, formatterResolver);
                        __transforms__b__ = true;
                        break;
                    case 3:
                        __twist__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]>().Deserialize(ref reader, formatterResolver);
                        __twist__b__ = true;
                        break;
                    case 4:
                        __wrench__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Wrench[]>().Deserialize(ref reader, formatterResolver);
                        __wrench__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiDOFJointState();
            if(__header__b__) ____result.header = __header__;
            if(__joint_names__b__) ____result.joint_names = __joint_names__;
            if(__transforms__b__) ____result.transforms = __transforms__;
            if(__twist__b__) ____result.twist = __twist__;
            if(__wrench__b__) ____result.wrench = __wrench__;

            return ____result;
        }
    }


    public sealed class MultiEchoLaserScanFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiEchoLaserScan>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MultiEchoLaserScanFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angle_min"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angle_max"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("angle_increment"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("time_increment"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("scan_time"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("range_min"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("range_max"), 7},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("ranges"), 8},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("intensities"), 9},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angle_min"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angle_max"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("angle_increment"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("time_increment"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("scan_time"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("range_min"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("range_max"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("ranges"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("intensities"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiEchoLaserScan value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteSingle(value.angle_min);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.angle_max);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteSingle(value.angle_increment);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteSingle(value.time_increment);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteSingle(value.scan_time);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteSingle(value.range_min);
            writer.WriteRaw(this.____stringByteKeys[7]);
            writer.WriteSingle(value.range_max);
            writer.WriteRaw(this.____stringByteKeys[8]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]>().Serialize(ref writer, value.ranges, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[9]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]>().Serialize(ref writer, value.intensities, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiEchoLaserScan Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __angle_min__ = default(float);
            var __angle_min__b__ = false;
            var __angle_max__ = default(float);
            var __angle_max__b__ = false;
            var __angle_increment__ = default(float);
            var __angle_increment__b__ = false;
            var __time_increment__ = default(float);
            var __time_increment__b__ = false;
            var __scan_time__ = default(float);
            var __scan_time__b__ = false;
            var __range_min__ = default(float);
            var __range_min__b__ = false;
            var __range_max__ = default(float);
            var __range_max__b__ = false;
            var __ranges__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]);
            var __ranges__b__ = false;
            var __intensities__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]);
            var __intensities__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __angle_min__ = reader.ReadSingle();
                        __angle_min__b__ = true;
                        break;
                    case 2:
                        __angle_max__ = reader.ReadSingle();
                        __angle_max__b__ = true;
                        break;
                    case 3:
                        __angle_increment__ = reader.ReadSingle();
                        __angle_increment__b__ = true;
                        break;
                    case 4:
                        __time_increment__ = reader.ReadSingle();
                        __time_increment__b__ = true;
                        break;
                    case 5:
                        __scan_time__ = reader.ReadSingle();
                        __scan_time__b__ = true;
                        break;
                    case 6:
                        __range_min__ = reader.ReadSingle();
                        __range_min__b__ = true;
                        break;
                    case 7:
                        __range_max__ = reader.ReadSingle();
                        __range_max__b__ = true;
                        break;
                    case 8:
                        __ranges__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]>().Deserialize(ref reader, formatterResolver);
                        __ranges__b__ = true;
                        break;
                    case 9:
                        __intensities__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.LaserEcho[]>().Deserialize(ref reader, formatterResolver);
                        __intensities__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.MultiEchoLaserScan();
            if(__header__b__) ____result.header = __header__;
            if(__angle_min__b__) ____result.angle_min = __angle_min__;
            if(__angle_max__b__) ____result.angle_max = __angle_max__;
            if(__angle_increment__b__) ____result.angle_increment = __angle_increment__;
            if(__time_increment__b__) ____result.time_increment = __time_increment__;
            if(__scan_time__b__) ____result.scan_time = __scan_time__;
            if(__range_min__b__) ____result.range_min = __range_min__;
            if(__range_max__b__) ____result.range_max = __range_max__;
            if(__ranges__b__) ____result.ranges = __ranges__;
            if(__intensities__b__) ____result.intensities = __intensities__;

            return ____result;
        }
    }


    public sealed class NavSatStatusFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public NavSatStatusFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("service"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("service"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteSByte(value.status);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt16(value.service);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __status__ = default(sbyte);
            var __status__b__ = false;
            var __service__ = default(ushort);
            var __service__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __status__ = reader.ReadSByte();
                        __status__b__ = true;
                        break;
                    case 1:
                        __service__ = reader.ReadUInt16();
                        __service__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus();
            if(__status__b__) ____result.status = __status__;
            if(__service__b__) ____result.service = __service__;

            return ____result;
        }
    }


    public sealed class NavSatFixFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatFix>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public NavSatFixFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("latitude"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("longitude"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("altitude"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("position_covariance"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("position_covariance_type"), 6},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("latitude"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("longitude"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("altitude"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("position_covariance"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("position_covariance_type"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatFix value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.latitude);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteDouble(value.longitude);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteDouble(value.altitude);
            writer.WriteRaw(this.____stringByteKeys[5]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.position_covariance, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteByte(value.position_covariance_type);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatFix Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus);
            var __status__b__ = false;
            var __latitude__ = default(double);
            var __latitude__b__ = false;
            var __longitude__ = default(double);
            var __longitude__b__ = false;
            var __altitude__ = default(double);
            var __altitude__b__ = false;
            var __position_covariance__ = default(double[]);
            var __position_covariance__b__ = false;
            var __position_covariance_type__ = default(byte);
            var __position_covariance_type__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __latitude__ = reader.ReadDouble();
                        __latitude__b__ = true;
                        break;
                    case 3:
                        __longitude__ = reader.ReadDouble();
                        __longitude__b__ = true;
                        break;
                    case 4:
                        __altitude__ = reader.ReadDouble();
                        __altitude__b__ = true;
                        break;
                    case 5:
                        __position_covariance__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __position_covariance__b__ = true;
                        break;
                    case 6:
                        __position_covariance_type__ = reader.ReadByte();
                        __position_covariance_type__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.NavSatFix();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__latitude__b__) ____result.latitude = __latitude__;
            if(__longitude__b__) ____result.longitude = __longitude__;
            if(__altitude__b__) ____result.altitude = __altitude__;
            if(__position_covariance__b__) ____result.position_covariance = __position_covariance__;
            if(__position_covariance_type__b__) ____result.position_covariance_type = __position_covariance_type__;

            return ____result;
        }
    }


    public sealed class PointCloudFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PointCloudFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("points"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("channels"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("points"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("channels"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]>().Serialize(ref writer, value.points, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32[]>().Serialize(ref writer, value.channels, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __points__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]);
            var __points__b__ = false;
            var __channels__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32[]);
            var __channels__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __points__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point32[]>().Deserialize(ref reader, formatterResolver);
                        __points__b__ = true;
                        break;
                    case 2:
                        __channels__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.ChannelFloat32[]>().Deserialize(ref reader, formatterResolver);
                        __channels__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud();
            if(__header__b__) ____result.header = __header__;
            if(__points__b__) ____result.points = __points__;
            if(__channels__b__) ____result.channels = __channels__;

            return ____result;
        }
    }


    public sealed class PointFieldFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PointFieldFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("name"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("offset"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("datatype"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("count"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("name"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("offset"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("datatype"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("count"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.name);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.offset);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteByte(value.datatype);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteUInt32(value.count);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __name__ = default(string);
            var __name__b__ = false;
            var __offset__ = default(uint);
            var __offset__b__ = false;
            var __datatype__ = default(byte);
            var __datatype__b__ = false;
            var __count__ = default(uint);
            var __count__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __name__ = reader.ReadString();
                        __name__b__ = true;
                        break;
                    case 1:
                        __offset__ = reader.ReadUInt32();
                        __offset__b__ = true;
                        break;
                    case 2:
                        __datatype__ = reader.ReadByte();
                        __datatype__b__ = true;
                        break;
                    case 3:
                        __count__ = reader.ReadUInt32();
                        __count__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField();
            if(__name__b__) ____result.name = __name__;
            if(__offset__b__) ____result.offset = __offset__;
            if(__datatype__b__) ____result.datatype = __datatype__;
            if(__count__b__) ____result.count = __count__;

            return ____result;
        }
    }


    public sealed class PointCloud2Formatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud2>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PointCloud2Formatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("height"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("width"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fields"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("is_bigendian"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("point_step"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("row_step"), 6},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("data"), 7},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("is_dense"), 8},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("height"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("width"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fields"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("is_bigendian"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("point_step"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("row_step"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("data"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("is_dense"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud2 value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteUInt32(value.height);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteUInt32(value.width);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField[]>().Serialize(ref writer, value.fields, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteBoolean(value.is_bigendian);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteUInt32(value.point_step);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteUInt32(value.row_step);
            writer.WriteRaw(this.____stringByteKeys[7]);
            formatterResolver.GetFormatterWithVerify<byte[]>().Serialize(ref writer, value.data, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[8]);
            writer.WriteBoolean(value.is_dense);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud2 Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __height__ = default(uint);
            var __height__b__ = false;
            var __width__ = default(uint);
            var __width__b__ = false;
            var __fields__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField[]);
            var __fields__b__ = false;
            var __is_bigendian__ = default(bool);
            var __is_bigendian__b__ = false;
            var __point_step__ = default(uint);
            var __point_step__b__ = false;
            var __row_step__ = default(uint);
            var __row_step__b__ = false;
            var __data__ = default(byte[]);
            var __data__b__ = false;
            var __is_dense__ = default(bool);
            var __is_dense__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __height__ = reader.ReadUInt32();
                        __height__b__ = true;
                        break;
                    case 2:
                        __width__ = reader.ReadUInt32();
                        __width__b__ = true;
                        break;
                    case 3:
                        __fields__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointField[]>().Deserialize(ref reader, formatterResolver);
                        __fields__b__ = true;
                        break;
                    case 4:
                        __is_bigendian__ = reader.ReadBoolean();
                        __is_bigendian__b__ = true;
                        break;
                    case 5:
                        __point_step__ = reader.ReadUInt32();
                        __point_step__b__ = true;
                        break;
                    case 6:
                        __row_step__ = reader.ReadUInt32();
                        __row_step__b__ = true;
                        break;
                    case 7:
                        __data__ = formatterResolver.GetFormatterWithVerify<byte[]>().Deserialize(ref reader, formatterResolver);
                        __data__b__ = true;
                        break;
                    case 8:
                        __is_dense__ = reader.ReadBoolean();
                        __is_dense__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.PointCloud2();
            if(__header__b__) ____result.header = __header__;
            if(__height__b__) ____result.height = __height__;
            if(__width__b__) ____result.width = __width__;
            if(__fields__b__) ____result.fields = __fields__;
            if(__is_bigendian__b__) ____result.is_bigendian = __is_bigendian__;
            if(__point_step__b__) ____result.point_step = __point_step__;
            if(__row_step__b__) ____result.row_step = __row_step__;
            if(__data__b__) ____result.data = __data__;
            if(__is_dense__b__) ____result.is_dense = __is_dense__;

            return ____result;
        }
    }


    public sealed class RangeFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Range>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public RangeFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("radiation_type"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("field_of_view"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("min_range"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("max_range"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("range"), 5},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("radiation_type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("field_of_view"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("min_range"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("max_range"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("range"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Range value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteByte(value.radiation_type);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteSingle(value.field_of_view);
            writer.WriteRaw(this.____stringByteKeys[3]);
            writer.WriteSingle(value.min_range);
            writer.WriteRaw(this.____stringByteKeys[4]);
            writer.WriteSingle(value.max_range);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteSingle(value.range);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Range Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __radiation_type__ = default(byte);
            var __radiation_type__b__ = false;
            var __field_of_view__ = default(float);
            var __field_of_view__b__ = false;
            var __min_range__ = default(float);
            var __min_range__b__ = false;
            var __max_range__ = default(float);
            var __max_range__b__ = false;
            var __range__ = default(float);
            var __range__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __radiation_type__ = reader.ReadByte();
                        __radiation_type__b__ = true;
                        break;
                    case 2:
                        __field_of_view__ = reader.ReadSingle();
                        __field_of_view__b__ = true;
                        break;
                    case 3:
                        __min_range__ = reader.ReadSingle();
                        __min_range__b__ = true;
                        break;
                    case 4:
                        __max_range__ = reader.ReadSingle();
                        __max_range__b__ = true;
                        break;
                    case 5:
                        __range__ = reader.ReadSingle();
                        __range__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Range();
            if(__header__b__) ____result.header = __header__;
            if(__radiation_type__b__) ____result.radiation_type = __radiation_type__;
            if(__field_of_view__b__) ____result.field_of_view = __field_of_view__;
            if(__min_range__b__) ____result.min_range = __min_range__;
            if(__max_range__b__) ____result.max_range = __max_range__;
            if(__range__b__) ____result.range = __range__;

            return ____result;
        }
    }


    public sealed class RelativeHumidityFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RelativeHumidity>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public RelativeHumidityFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("relative_humidity"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("variance"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("relative_humidity"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("variance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RelativeHumidity value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.relative_humidity);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.variance);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RelativeHumidity Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __relative_humidity__ = default(double);
            var __relative_humidity__b__ = false;
            var __variance__ = default(double);
            var __variance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __relative_humidity__ = reader.ReadDouble();
                        __relative_humidity__b__ = true;
                        break;
                    case 2:
                        __variance__ = reader.ReadDouble();
                        __variance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.RelativeHumidity();
            if(__header__b__) ____result.header = __header__;
            if(__relative_humidity__b__) ____result.relative_humidity = __relative_humidity__;
            if(__variance__b__) ____result.variance = __variance__;

            return ____result;
        }
    }


    public sealed class TemperatureFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Temperature>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TemperatureFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("temperature"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("variance"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("temperature"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("variance"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Temperature value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteDouble(value.temperature);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteDouble(value.variance);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Temperature Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __temperature__ = default(double);
            var __temperature__b__ = false;
            var __variance__ = default(double);
            var __variance__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __temperature__ = reader.ReadDouble();
                        __temperature__b__ = true;
                        break;
                    case 2:
                        __variance__ = reader.ReadDouble();
                        __variance__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.Temperature();
            if(__header__b__) ____result.header = __header__;
            if(__temperature__b__) ____result.temperature = __temperature__;
            if(__variance__b__) ____result.variance = __variance__;

            return ____result;
        }
    }


    public sealed class TimeReferenceFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.TimeReference>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TimeReferenceFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("time_ref"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("source"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("time_ref"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("source"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.TimeReference value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.time_ref, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            writer.WriteString(value.source);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.TimeReference Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __time_ref__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __time_ref__b__ = false;
            var __source__ = default(string);
            var __source__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __time_ref__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __time_ref__b__ = true;
                        break;
                    case 2:
                        __source__ = reader.ReadString();
                        __source__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.TimeReference();
            if(__header__b__) ____result.header = __header__;
            if(__time_ref__b__) ____result.time_ref = __time_ref__;
            if(__source__b__) ____result.source = __source__;

            return ____result;
        }
    }


    public sealed class SetCameraInfoRequestFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoRequest>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetCameraInfoRequestFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("camera_info"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("camera_info"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoRequest value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo>().Serialize(ref writer, value.camera_info, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoRequest Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __camera_info__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo);
            var __camera_info__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __camera_info__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo>().Deserialize(ref reader, formatterResolver);
                        __camera_info__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoRequest();
            if(__camera_info__b__) ____result.camera_info = __camera_info__;

            return ____result;
        }
    }


    public sealed class SetCameraInfoResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SetCameraInfoResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("success"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status_message"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("success"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status_message"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteBoolean(value.success);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.status_message);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __success__ = default(bool);
            var __success__b__ = false;
            var __status_message__ = default(string);
            var __status_message__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __success__ = reader.ReadBoolean();
                        __success__b__ = true;
                        break;
                    case 1:
                        __status_message__ = reader.ReadString();
                        __status_message__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Sensor.SetCameraInfoResponse();
            if(__success__b__) ____result.success = __success__;
            if(__status_message__b__) ____result.status_message = __status_message__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Shape
{
    using System;
    using Utf8Json;


    public sealed class MeshTriangleFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MeshTriangleFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("vertex_indices"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("vertex_indices"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<uint[]>().Serialize(ref writer, value.vertex_indices, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __vertex_indices__ = default(uint[]);
            var __vertex_indices__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __vertex_indices__ = formatterResolver.GetFormatterWithVerify<uint[]>().Deserialize(ref reader, formatterResolver);
                        __vertex_indices__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle();
            if(__vertex_indices__b__) ____result.vertex_indices = __vertex_indices__;

            return ____result;
        }
    }


    public sealed class MeshFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Shape.Mesh>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MeshFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("triangles"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("vertices"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("triangles"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("vertices"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Shape.Mesh value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle[]>().Serialize(ref writer, value.triangles, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]>().Serialize(ref writer, value.vertices, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Shape.Mesh Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __triangles__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle[]);
            var __triangles__b__ = false;
            var __vertices__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]);
            var __vertices__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __triangles__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Shape.MeshTriangle[]>().Deserialize(ref reader, formatterResolver);
                        __triangles__b__ = true;
                        break;
                    case 1:
                        __vertices__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Point[]>().Deserialize(ref reader, formatterResolver);
                        __vertices__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Shape.Mesh();
            if(__triangles__b__) ____result.triangles = __triangles__;
            if(__vertices__b__) ____result.vertices = __vertices__;

            return ____result;
        }
    }


    public sealed class PlaneFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Shape.Plane>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public PlaneFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("coef"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("coef"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Shape.Plane value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.coef, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Shape.Plane Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __coef__ = default(double[]);
            var __coef__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __coef__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __coef__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Shape.Plane();
            if(__coef__b__) ____result.coef = __coef__;

            return ____result;
        }
    }


    public sealed class SolidPrimitiveFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Shape.SolidPrimitive>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public SolidPrimitiveFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("type"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("dimensions"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("type"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("dimensions"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Shape.SolidPrimitive value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteByte(value.type);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.dimensions, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Shape.SolidPrimitive Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __type__ = default(byte);
            var __type__b__ = false;
            var __dimensions__ = default(double[]);
            var __dimensions__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __type__ = reader.ReadByte();
                        __type__b__ = true;
                        break;
                    case 1:
                        __dimensions__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __dimensions__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Shape.SolidPrimitive();
            if(__type__b__) ____result.type = __type__;
            if(__dimensions__b__) ____result.dimensions = __dimensions__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Tf2
{
    using System;
    using Utf8Json;


    public sealed class LookupTransformGoalFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LookupTransformGoalFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("target_frame"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("source_frame"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("source_time"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("timeout"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("target_time"), 4},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("fixed_frame"), 5},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("advanced"), 6},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("target_frame"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("source_frame"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("source_time"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("timeout"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("target_time"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("fixed_frame"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("advanced"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.target_frame);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.source_frame);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.source_time, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>().Serialize(ref writer, value.timeout, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Serialize(ref writer, value.target_time, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[5]);
            writer.WriteString(value.fixed_frame);
            writer.WriteRaw(this.____stringByteKeys[6]);
            writer.WriteBoolean(value.advanced);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __target_frame__ = default(string);
            var __target_frame__b__ = false;
            var __source_frame__ = default(string);
            var __source_frame__b__ = false;
            var __source_time__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __source_time__b__ = false;
            var __timeout__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration);
            var __timeout__b__ = false;
            var __target_time__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Time);
            var __target_time__b__ = false;
            var __fixed_frame__ = default(string);
            var __fixed_frame__b__ = false;
            var __advanced__ = default(bool);
            var __advanced__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __target_frame__ = reader.ReadString();
                        __target_frame__b__ = true;
                        break;
                    case 1:
                        __source_frame__ = reader.ReadString();
                        __source_frame__b__ = true;
                        break;
                    case 2:
                        __source_time__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __source_time__b__ = true;
                        break;
                    case 3:
                        __timeout__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>().Deserialize(ref reader, formatterResolver);
                        __timeout__b__ = true;
                        break;
                    case 4:
                        __target_time__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Time>().Deserialize(ref reader, formatterResolver);
                        __target_time__b__ = true;
                        break;
                    case 5:
                        __fixed_frame__ = reader.ReadString();
                        __fixed_frame__b__ = true;
                        break;
                    case 6:
                        __advanced__ = reader.ReadBoolean();
                        __advanced__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal();
            if(__target_frame__b__) ____result.target_frame = __target_frame__;
            if(__source_frame__b__) ____result.source_frame = __source_frame__;
            if(__source_time__b__) ____result.source_time = __source_time__;
            if(__timeout__b__) ____result.timeout = __timeout__;
            if(__target_time__b__) ____result.target_time = __target_time__;
            if(__fixed_frame__b__) ____result.fixed_frame = __fixed_frame__;
            if(__advanced__b__) ____result.advanced = __advanced__;

            return ____result;
        }
    }


    public sealed class LookupTransformActionGoalFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LookupTransformActionGoalFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal_id"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("goal"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal_id"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("goal"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Serialize(ref writer, value.goal_id, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal>().Serialize(ref writer, value.goal, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __goal_id__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID);
            var __goal_id__b__ = false;
            var __goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal);
            var __goal__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __goal_id__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalID>().Deserialize(ref reader, formatterResolver);
                        __goal_id__b__ = true;
                        break;
                    case 2:
                        __goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformGoal>().Deserialize(ref reader, formatterResolver);
                        __goal__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal();
            if(__header__b__) ____result.header = __header__;
            if(__goal_id__b__) ____result.goal_id = __goal_id__;
            if(__goal__b__) ____result.goal = __goal__;

            return ____result;
        }
    }


    public sealed class TF2ErrorFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TF2ErrorFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("error"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("error_string"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("error"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("error_string"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteByte(value.error);
            writer.WriteRaw(this.____stringByteKeys[1]);
            writer.WriteString(value.error_string);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __error__ = default(byte);
            var __error__b__ = false;
            var __error_string__ = default(string);
            var __error_string__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __error__ = reader.ReadByte();
                        __error__b__ = true;
                        break;
                    case 1:
                        __error_string__ = reader.ReadString();
                        __error_string__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error();
            if(__error__b__) ____result.error = __error__;
            if(__error_string__b__) ____result.error_string = __error_string__;

            return ____result;
        }
    }


    public sealed class LookupTransformResultFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LookupTransformResultFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("transform"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("error"), 1},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("transform"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("error"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped>().Serialize(ref writer, value.transform, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error>().Serialize(ref writer, value.error, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __transform__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped);
            var __transform__b__ = false;
            var __error__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error);
            var __error__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __transform__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped>().Deserialize(ref reader, formatterResolver);
                        __transform__b__ = true;
                        break;
                    case 1:
                        __error__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TF2Error>().Deserialize(ref reader, formatterResolver);
                        __error__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult();
            if(__transform__b__) ____result.transform = __transform__;
            if(__error__b__) ____result.error = __error__;

            return ____result;
        }
    }


    public sealed class LookupTransformActionResultFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LookupTransformActionResultFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("result"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("result"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult>().Serialize(ref writer, value.result, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus);
            var __status__b__ = false;
            var __result__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult);
            var __result__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __result__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformResult>().Deserialize(ref reader, formatterResolver);
                        __result__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__result__b__) ____result.result = __result__;

            return ____result;
        }
    }


    public sealed class LookupTransformActionFeedbackFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LookupTransformActionFeedbackFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("status"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("feedback"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("status"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("feedback"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Serialize(ref writer, value.status, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformFeedback>().Serialize(ref writer, value.feedback, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __status__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus);
            var __status__b__ = false;
            var __feedback__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformFeedback);
            var __feedback__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __status__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus>().Deserialize(ref reader, formatterResolver);
                        __status__b__ = true;
                        break;
                    case 2:
                        __feedback__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformFeedback>().Deserialize(ref reader, formatterResolver);
                        __feedback__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback();
            if(__header__b__) ____result.header = __header__;
            if(__status__b__) ____result.status = __status__;
            if(__feedback__b__) ____result.feedback = __feedback__;

            return ____result;
        }
    }


    public sealed class LookupTransformActionFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformAction>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public LookupTransformActionFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_goal"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_result"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("action_feedback"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("action_goal"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("action_result"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("action_feedback"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformAction value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal>().Serialize(ref writer, value.action_goal, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult>().Serialize(ref writer, value.action_result, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback>().Serialize(ref writer, value.action_feedback, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformAction Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __action_goal__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal);
            var __action_goal__b__ = false;
            var __action_result__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult);
            var __action_result__b__ = false;
            var __action_feedback__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback);
            var __action_feedback__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __action_goal__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionGoal>().Deserialize(ref reader, formatterResolver);
                        __action_goal__b__ = true;
                        break;
                    case 1:
                        __action_result__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionResult>().Deserialize(ref reader, formatterResolver);
                        __action_result__b__ = true;
                        break;
                    case 2:
                        __action_feedback__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformActionFeedback>().Deserialize(ref reader, formatterResolver);
                        __action_feedback__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.LookupTransformAction();
            if(__action_goal__b__) ____result.action_goal = __action_goal__;
            if(__action_result__b__) ____result.action_result = __action_result__;
            if(__action_feedback__b__) ____result.action_feedback = __action_feedback__;

            return ____result;
        }
    }


    public sealed class TFMessageFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TFMessage>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public TFMessageFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("transforms"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("transforms"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TFMessage value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped[]>().Serialize(ref writer, value.transforms, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TFMessage Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __transforms__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped[]);
            var __transforms__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __transforms__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped[]>().Deserialize(ref reader, formatterResolver);
                        __transforms__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.TFMessage();
            if(__transforms__b__) ____result.transforms = __transforms__;

            return ____result;
        }
    }


    public sealed class FrameGraphResponseFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Tf2.FrameGraphResponse>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public FrameGraphResponseFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("frame_yaml"), 0},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("frame_yaml"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Tf2.FrameGraphResponse value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            writer.WriteString(value.frame_yaml);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Tf2.FrameGraphResponse Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __frame_yaml__ = default(string);
            var __frame_yaml__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __frame_yaml__ = reader.ReadString();
                        __frame_yaml__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Tf2.FrameGraphResponse();
            if(__frame_yaml__b__) ____result.frame_yaml = __frame_yaml__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
#pragma warning disable 618
#pragma warning disable 612
#pragma warning disable 414
#pragma warning disable 219
#pragma warning disable 168

namespace Utf8Json.Formatters.RosSharp.RosBridgeClient.MessageTypes.Trajectory
{
    using System;
    using Utf8Json;


    public sealed class JointTrajectoryPointFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public JointTrajectoryPointFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("positions"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("velocities"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("accelerations"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("effort"), 3},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("time_from_start"), 4},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("positions"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("velocities"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("accelerations"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("effort"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("time_from_start"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.positions, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.velocities, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.accelerations, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<double[]>().Serialize(ref writer, value.effort, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[4]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>().Serialize(ref writer, value.time_from_start, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __positions__ = default(double[]);
            var __positions__b__ = false;
            var __velocities__ = default(double[]);
            var __velocities__b__ = false;
            var __accelerations__ = default(double[]);
            var __accelerations__b__ = false;
            var __effort__ = default(double[]);
            var __effort__b__ = false;
            var __time_from_start__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration);
            var __time_from_start__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __positions__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __positions__b__ = true;
                        break;
                    case 1:
                        __velocities__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __velocities__b__ = true;
                        break;
                    case 2:
                        __accelerations__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __accelerations__b__ = true;
                        break;
                    case 3:
                        __effort__ = formatterResolver.GetFormatterWithVerify<double[]>().Deserialize(ref reader, formatterResolver);
                        __effort__b__ = true;
                        break;
                    case 4:
                        __time_from_start__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>().Deserialize(ref reader, formatterResolver);
                        __time_from_start__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint();
            if(__positions__b__) ____result.positions = __positions__;
            if(__velocities__b__) ____result.velocities = __velocities__;
            if(__accelerations__b__) ____result.accelerations = __accelerations__;
            if(__effort__b__) ____result.effort = __effort__;
            if(__time_from_start__b__) ____result.time_from_start = __time_from_start__;

            return ____result;
        }
    }


    public sealed class JointTrajectoryFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectory>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public JointTrajectoryFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("joint_names"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("points"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("joint_names"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("points"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectory value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.joint_names, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint[]>().Serialize(ref writer, value.points, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectory Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __joint_names__ = default(string[]);
            var __joint_names__b__ = false;
            var __points__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint[]);
            var __points__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __joint_names__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __joint_names__b__ = true;
                        break;
                    case 2:
                        __points__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectoryPoint[]>().Deserialize(ref reader, formatterResolver);
                        __points__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.JointTrajectory();
            if(__header__b__) ____result.header = __header__;
            if(__joint_names__b__) ____result.joint_names = __joint_names__;
            if(__points__b__) ____result.points = __points__;

            return ____result;
        }
    }


    public sealed class MultiDOFJointTrajectoryPointFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MultiDOFJointTrajectoryPointFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("transforms"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("velocities"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("accelerations"), 2},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("time_from_start"), 3},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("transforms"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("velocities"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("accelerations"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("time_from_start"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]>().Serialize(ref writer, value.transforms, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]>().Serialize(ref writer, value.velocities, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]>().Serialize(ref writer, value.accelerations, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[3]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>().Serialize(ref writer, value.time_from_start, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __transforms__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]);
            var __transforms__b__ = false;
            var __velocities__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]);
            var __velocities__b__ = false;
            var __accelerations__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]);
            var __accelerations__b__ = false;
            var __time_from_start__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration);
            var __time_from_start__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __transforms__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform[]>().Deserialize(ref reader, formatterResolver);
                        __transforms__b__ = true;
                        break;
                    case 1:
                        __velocities__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]>().Deserialize(ref reader, formatterResolver);
                        __velocities__b__ = true;
                        break;
                    case 2:
                        __accelerations__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist[]>().Deserialize(ref reader, formatterResolver);
                        __accelerations__b__ = true;
                        break;
                    case 3:
                        __time_from_start__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Duration>().Deserialize(ref reader, formatterResolver);
                        __time_from_start__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint();
            if(__transforms__b__) ____result.transforms = __transforms__;
            if(__velocities__b__) ____result.velocities = __velocities__;
            if(__accelerations__b__) ____result.accelerations = __accelerations__;
            if(__time_from_start__b__) ____result.time_from_start = __time_from_start__;

            return ____result;
        }
    }


    public sealed class MultiDOFJointTrajectoryFormatter : global::Utf8Json.IJsonFormatter<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectory>
    {
        readonly global::Utf8Json.Internal.AutomataDictionary ____keyMapping;
        readonly byte[][] ____stringByteKeys;

        public MultiDOFJointTrajectoryFormatter()
        {
            this.____keyMapping = new global::Utf8Json.Internal.AutomataDictionary()
            {
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("header"), 0},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("joint_names"), 1},
                { JsonWriter.GetEncodedPropertyNameWithoutQuotation("points"), 2},
            };

            this.____stringByteKeys = new byte[][]
            {
                JsonWriter.GetEncodedPropertyNameWithBeginObject("header"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("joint_names"),
                JsonWriter.GetEncodedPropertyNameWithPrefixValueSeparator("points"),
                
            };
        }

        public void Serialize(ref JsonWriter writer, global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectory value, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (value == null)
            {
                writer.WriteNull();
                return;
            }
            

            writer.WriteRaw(this.____stringByteKeys[0]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Serialize(ref writer, value.header, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[1]);
            formatterResolver.GetFormatterWithVerify<string[]>().Serialize(ref writer, value.joint_names, formatterResolver);
            writer.WriteRaw(this.____stringByteKeys[2]);
            formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint[]>().Serialize(ref writer, value.points, formatterResolver);
            
            writer.WriteEndObject();
        }

        public global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectory Deserialize(ref JsonReader reader, global::Utf8Json.IJsonFormatterResolver formatterResolver)
        {
            if (reader.ReadIsNull())
            {
                return null;
            }
            

            var __header__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Std.Header);
            var __header__b__ = false;
            var __joint_names__ = default(string[]);
            var __joint_names__b__ = false;
            var __points__ = default(global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint[]);
            var __points__b__ = false;

            var ____count = 0;
            reader.ReadIsBeginObjectWithVerify();
            while (!reader.ReadIsEndObjectWithSkipValueSeparator(ref ____count))
            {
                var stringKey = reader.ReadPropertyNameSegmentRaw();
                int key;
                if (!____keyMapping.TryGetValueSafe(stringKey, out key))
                {
                    reader.ReadNextBlock();
                    goto NEXT_LOOP;
                }

                switch (key)
                {
                    case 0:
                        __header__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Std.Header>().Deserialize(ref reader, formatterResolver);
                        __header__b__ = true;
                        break;
                    case 1:
                        __joint_names__ = formatterResolver.GetFormatterWithVerify<string[]>().Deserialize(ref reader, formatterResolver);
                        __joint_names__b__ = true;
                        break;
                    case 2:
                        __points__ = formatterResolver.GetFormatterWithVerify<global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectoryPoint[]>().Deserialize(ref reader, formatterResolver);
                        __points__b__ = true;
                        break;
                    default:
                        reader.ReadNextBlock();
                        break;
                }

                NEXT_LOOP:
                continue;
            }

            var ____result = new global::RosSharp.RosBridgeClient.MessageTypes.Trajectory.MultiDOFJointTrajectory();
            if(__header__b__) ____result.header = __header__;
            if(__joint_names__b__) ____result.joint_names = __joint_names__;
            if(__points__b__) ____result.points = __points__;

            return ____result;
        }
    }

}

#pragma warning disable 168
#pragma warning restore 219
#pragma warning restore 414
#pragma warning restore 618
#pragma warning restore 612
