using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClockPublisher : UnityPublisher<Clock>
{
    int counter = 0;


    private void Update()
    {
        if(counter++ % 50 == 0)
        {
            counter = 0;
            Publish(new Clock()
            {
                clock = new RosSharp.RosBridgeClient.MessageTypes.Std.Time()
                {
                    nsecs = 10,
                    secs = (uint)Time.realtimeSinceStartup,
                }
            });
        }
    }
}
