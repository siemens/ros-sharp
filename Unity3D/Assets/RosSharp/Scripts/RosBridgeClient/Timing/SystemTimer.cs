/*
Shimadzu corp , 2019, Akira NODA (a-noda@shimadzu.co.jp / you.akira.noda@gmail.com)

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

// Please check out https://gist.github.com/akirayou/0a444f0081acd1f268804af0347957f9
// for a NTP implementation of the Timer class


using System;

namespace RosSharp.RosBridgeClient
{
    public class SystemTimer : Timer
    {
        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
        public override Messages.Standard.Time Now()
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds;
            uint sec = (uint)(msecs / 1000);

            return new Messages.Standard.Time
            {
                secs = sec,
                nsecs = (uint)((msecs / 1000 - sec) * 1e+9)
            };
        }
    }
}