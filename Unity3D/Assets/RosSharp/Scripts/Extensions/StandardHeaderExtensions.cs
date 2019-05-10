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

// Adding Timestamp switching
// Shimadzu corp , 2019, Akira NODA (a-noda@shimadzu.co.jp / you.akira.noda@gmail.com)

namespace RosSharp.RosBridgeClient
{
    public interface IRosTimeNow
    {
        Messages.Standard.Time Now();
    }
    public class DefaultRosTimeNow:IRosTimeNow
    {
        public Messages.Standard.Time Now()
        {
            Messages.Standard.Time stamp = new Messages.Standard.Time();
            float time = UnityEngine.Time.realtimeSinceStartup;
            uint secs = (uint)time;
            uint nsecs = (uint)(1e9 * (time - secs));
            stamp.secs = secs;
            stamp.nsecs = nsecs;
            return stamp;
        }
    };
    public static class HeaderExtensions
    {
        private static IRosTimeNow rosTimeNow = null;
        private static DefaultRosTimeNow defaultRosTimeNow = new DefaultRosTimeNow();
        public static IRosTimeNow RosTimeNow { set { rosTimeNow = value; } }
        static HeaderExtensions()
        {
            rosTimeNow = defaultRosTimeNow;
        } 
        public static void Update(this Messages.Standard.Header header)
        {
            header.seq++;
            header.stamp =rosTimeNow.Now();
        }
    }
}
