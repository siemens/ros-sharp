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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.Net;
using System.Net.Sockets;
using GuerrillaNtp;

using System.Threading.Tasks;

namespace RosSharp.RosBridgeClient
{
    /// <summary>
    /// Use NTP for Message header
    /// To use this. Just attach to Any object object in Unity
    /// and configure host and port.
    /// </summary>
    public class NtpTime : MonoBehaviour,IRosTimeNow
    {
        /// <summary>
        /// NTP server host name
        /// </summary>
        public string Host = "10.0.1.108";
        /// <summary>
        /// NTP port (default:123)
        /// </summary>
        public Int16 Port = 123;
        /// <summary>
        /// NTP request time span (sec)
        /// </summary>
        public float NtpSpan = 10;
        /// <summary>
        /// Just for Application (not for RosBridge) Check time sync was done 
        /// </summary>
        /// <returns></returns>
        public bool Synced() { return synced; }
        public TimeSpan Offset() { return offset; }

        private TimeSpan offset;
        private bool synced = false;

        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
        public Messages.Standard.Time Now()
        {
            TimeSpan unixEpoch = DateTime.Now.ToUniversalTime() - UNIX_EPOCH + offset;
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
            NtpClient ntp;
            ntp = new NtpClient(Dns.GetHostEntry(Host).AddressList[0], Port);
            Debug.Log("Use NTP time for Ros header timestamp");

        }

        // Update is called once per frame
        private float timeElapsed=1e+10f;
        async void Update()
        {
            timeElapsed += Time.deltaTime;
            if (timeElapsed < NtpSpan) return;
            timeElapsed = 0;
            TimeSpan tmp;
            bool isOk = false;
            await Task.Run(() =>
            {
                try
                {
                    NtpClient ntp;
                    ntp = new NtpClient(Dns.GetHostEntry(Host).AddressList[0], Port);
                    tmp = ntp.GetCorrectionOffset();
                    isOk = true;
                }
                catch (Exception e)
                {
                    Debug.LogError("Ntp error:" + e.ToString());
                    isOk = false;
                }
            });
            if (!isOk) return;
            if (!synced)
            {
                offset = tmp;
                synced = true;
                Debug.Log("NtpTime is started  offset is "+offset.ToString());
            }
            else
            {
                offset = TimeSpan.FromSeconds(0.9 * offset.TotalSeconds + 0.1 * tmp.TotalSeconds);
            }
        }
    }
}