/*
© Siemens AG, 2017-2019
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

// Adding Timestamp switching
// Shimadzu corp , 2019, Akira NODA (a-noda@shimadzu.co.jp / you.akira.noda@gmail.com)

// Removing MonoBehaviour inheritance
// Siemens AG , 2019, Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com) 

// Added allocation free alternatives
// UoK , 2019, Odysseas Doumas (od79@kent.ac.uk / odydoum@gmail.com) 

// Added preprocessor directive flags for ROS2 support
// Siemens AG , 2024, Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com) 

using System;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class Timer
    {
        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

#if !ROS2
        public virtual MessageTypes.Std.Time Now()
        {
            Now(out uint secs, out uint nsecs);
            return new MessageTypes.Std.Time(secs, nsecs);
        }

        public virtual void Now(MessageTypes.Std.Time stamp)
        {
            uint secs; uint nsecs;
            Now(out secs, out nsecs);
            stamp.secs = secs; stamp.nsecs = nsecs;
        }

        private static void Now(out uint secs, out uint nsecs)
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds;
            secs = (uint)(msecs / 1000);
            nsecs = (uint)((msecs / 1000 - secs) * 1e+9);
        }
#else
        public virtual MessageTypes.Std.Time Now()
        {
            Now(out uint sec, out uint nanosec);
            return new MessageTypes.Std.Time(sec, nanosec);
        }

        public virtual void Now(MessageTypes.Std.Time stamp)
        {
            uint sec; uint nanosec;
            Now(out sec, out nanosec);
            stamp.sec = sec; stamp.nanosec = nanosec;
        }

        private static void Now(out uint sec, out uint nanosec)
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds;
            sec = (uint)(msecs / 1000);
            nanosec = (uint)((msecs / 1000 - sec) * 1e+9);
        }
#endif    
    }
}
