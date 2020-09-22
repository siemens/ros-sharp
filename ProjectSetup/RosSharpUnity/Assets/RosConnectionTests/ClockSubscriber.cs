using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ClockSubscriber : UnitySubscriber<Clock>
{
    public TextMeshPro TextElement;
    private string LastMsg;
    private bool receivedMsg = false;
    protected override void ReceiveMessage(Clock message)
    {
        receivedMsg = true;
        LastMsg = "current ROS clock time: " + message.clock.secs + " secs";
    }
    public void Update()
    {
        if(TextElement != null && receivedMsg)
        {
            TextElement.text = LastMsg;
            receivedMsg = true;
        }
    }
}

public class Clock : RosSharp.RosBridgeClient.Message
{
    public override string RosMessageName => "rosgraph_msgs/Clock";

    public RosSharp.RosBridgeClient.MessageTypes.Std.Time clock { get; set; }

    public Clock()
    {

    }
}