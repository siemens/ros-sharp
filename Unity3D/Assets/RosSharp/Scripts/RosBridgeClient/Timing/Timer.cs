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

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Timer: MonoBehaviour
    {
        protected void Awake()
        {
            HeaderExtensions.Timer = this;
        }

        public virtual Messages.Standard.Time Now()
        {
            Messages.Standard.Time stamp = new Messages.Standard.Time();
            float time = Time.realtimeSinceStartup;
            stamp.secs = (uint)time;
            stamp.nsecs = (uint)(1e9 * (time - stamp.secs));
            return stamp;
        }
    }
}
