/*
© ASTRA - of the Space Hardware Club at UAH, 2026
Author: Roald Schaum, roaldschaum2019@gmail.com

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

namespace RosSharp.RosBridgeClient
{
#if ROS2
    public record QOS(string history, uint depth, string reliability, string durability, Duration deadline, Duration lifespan)
    {
        public QOS(QOS q)
        {
            history = q.history;
            depth = q.depth;
            reliability = q.reliability;
            durability = q.durability;
            deadline = q.deadline;
            lifespan = q.lifespan;
        }

        public QOS(
            Policy.History history,
            uint depth,
            Policy.Reliability reliability,
            Policy.Durability durability,
            double deadline,
            double lifespan) :
        this(
                Enum.GetName(history),
                depth,
                Enum.GetName(reliability),
                Enum.GetName(durability),
                Duration.DoubleToTime(deadline),
                Duration.DoubleToTime(lifespan))
        { }

        public QOS(
            Policy.History history,
            uint depth,
            Policy.Reliability reliability,
            Policy.Durability durability,
            Duration deadline,
            Duration lifespan) :
        this(
            Enum.GetName(history),
            depth,
            Enum.GetName(reliability),
            Enum.GetName(durability),
            deadline,
            lifespan)
        { }

        /// <summary> Default QOS profiles as specified by the RMW qos_profiles.h file </summary>
        public class Presets
        {
            /// <summary> Default QOS profile </summary>
            public static readonly QOS Default = new(
                Policy.History.KEEP_LAST,
                10,
                Policy.Reliability.RELIABLE,
                Policy.Durability.VOLATILE,
                Policy.Duration_Unspecified,
                Policy.Duration_Unspecified);

            /// <summary> Default Sensor QOS profile </summary>
            public static readonly QOS SensorData = new(
                Policy.History.KEEP_LAST,
                5,
                Policy.Reliability.BEST_EFFORT,
                Policy.Durability.VOLATILE,
                Policy.Duration_Unspecified,
                Policy.Duration_Unspecified);

            /// <summary> Default Sensor QOS profile </summary>
            public static readonly QOS Services = new(
                Policy.History.KEEP_LAST,
                10,
                Policy.Reliability.RELIABLE,
                Policy.Durability.VOLATILE,
                Policy.Duration_Unspecified,
                Policy.Duration_Unspecified);

            /// <summary> Default Sensor QOS profile </summary>
            public static readonly QOS ParameterEvents = new(
                Policy.History.KEEP_LAST,
                1000,
                Policy.Reliability.RELIABLE,
                Policy.Durability.VOLATILE,
                Policy.Duration_Unspecified,
                Policy.Duration_Unspecified);

            /// <summary> Default Sensor QOS profile </summary>
            public static readonly QOS Rosout = new(
                Policy.History.KEEP_LAST,
                1000,
                Policy.Reliability.RELIABLE,
                Policy.Durability.TRANSIENT_LOCAL,
                Policy.Duration_Unspecified,
                new Duration(10, 0));

            /// <summary> Default Sensor QOS profile </summary>
            public static readonly QOS SystemDefault = new(
                Policy.History.SYSTEM_DEFAULT,
                0,
                Policy.Reliability.SYSTEM_DEFAULT,
                Policy.Durability.SYSTEM_DEFAULT,
                Policy.Duration_Unspecified,
                Policy.Duration_Unspecified);
        }

        public static class Policy
        {
            public static Duration Duration_Unspecified
            {
                get;
                private set;
            } = new(0ul, 0ul);
            public static Duration Duration_Infinite
            {
                get;
                private set;
            } = new(9223372036ul, 854775807ul);

            public enum History
            {
                /// <summary> Implementation default for history policy </summary>
                SYSTEM_DEFAULT,

                /// <summary> Only store up to a maximum number of samples, dropping oldest once max is exceeded </summary>
                KEEP_LAST,

                /// <summary> Store all samples, subject to resource limits </summary>
                KEEP_ALL,
            }
            public enum Reliability
            {
                /// <summary> Implementation specific default </summary>
                SYSTEM_DEFAULT,

                /// <summary> Guarantee that samples are delivered, may retry multiple times </summary>
                RELIABLE,

                /// <summary> Attempt to deliver samples, but some may be lost if the network is not robust </summary>
                BEST_EFFORT,
            }

            public enum Durability
            {
                /// <summary> Implementation specific default </summary>
                SYSTEM_DEFAULT,

                /// <summary> The rmw publisher is responsible for persisting samples for “late-joining” subscribers </summary>
                TRANSIENT_LOCAL,

                /// <summary> Samples are not persistent </summary>
                VOLATILE,
            }
        }

    }
    public record Duration(ulong secs = 0, ulong nsecs = 0)
    {
        public static Duration DoubleToTime(double time)
        {
            const ulong kcubed = 1000 * 1000 * 1000;
            double ftime = Math.Floor(time);
            return new Duration((ulong)ftime, (ulong)((time - ftime) * kcubed));
        }
    }
#endif
}
