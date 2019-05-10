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