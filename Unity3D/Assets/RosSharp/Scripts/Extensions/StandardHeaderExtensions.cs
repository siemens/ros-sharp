﻿/*
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

namespace RosSharp.RosBridgeClient
{
    public static class HeaderExtensions
    {
        public static void Update(this Messages.Standard.Header header)
        {
            float time = UnityEngine.Time.realtimeSinceStartup;
            uint secs = (uint)time;
            uint nsecs = (uint)(1e9 *(time-secs));
            header.seq++;
            header.stamp.secs = secs;
            header.stamp.nsecs = nsecs;
        }
    }
}
