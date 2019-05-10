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

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    /// <summary>
    /// Use System Time for Message header
    /// To use this. Just attach to Any object object in Unity
    /// </summary>
    public class SystemTime : MonoBehaviour,IRosTimeNow
    {
        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
        public Messages.Standard.Time Now()
        {
            TimeSpan unixEpoch = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double ds = unixEpoch.TotalMilliseconds;
            uint sec = (uint)(ds / 1000);
            RosSharp.RosBridgeClient.Messages.Standard.Time ret = new RosSharp.RosBridgeClient.Messages.Standard.Time
            {
                secs = sec,
                nsecs = (uint)((ds / 1000 - sec) * 1e+9)
            };
            return ret;
        }

        void Awake()
        {
            //Override time stamp header
            HeaderExtensions.RosTimeNow = this;
            Debug.Log("Use sytem time for Ros header timestamp");
        }

        void Start()
        {
        }
        void Update()
        {
        }
    }
}