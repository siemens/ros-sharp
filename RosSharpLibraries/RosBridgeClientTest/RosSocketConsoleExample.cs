using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages;
using Newtonsoft.Json.Linq;

// commands on ROS system:
// launch before starting:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /publication_test
// rostopic pub /subscription_test std_msgs/String "subscription test message data"

// launch after starting:
// rosservice call /service_response_test

public class RosSocketConsole
{
    private static string url = "ws://192.168.56.102:9090";

    public static void Main(string[] args)
    {
        RosSocket rosSocket = new RosSocket(url);

        // Publication:
        string publication_id = rosSocket.Advertise("/publication_test", "std_msgs/String");
        StandardString message = new StandardString("publication test message data");
        rosSocket.Publish(publication_id, message);

        // Subscription:
        string subscription_id = rosSocket.Subscribe("/subscription_test",typeof(StandardString), SubscriptionHandler);

        // Service Call:
        rosSocket.CallService("/rosapi/get_param", typeof(RosApiGetParamResponse), ServiceCallHandler, new RosApiGetParamRequest("/rosdistro"));

        // Service Response:
        rosSocket.AdvertiseService("/service_response_test", typeof(StandardServiceTriggerRequest), ServiceResponseHandler);

        Console.WriteLine("Press any key to close...");
        Console.ReadKey(true);
        rosSocket.Close();
    }
    private static void SubscriptionHandler(Message message)
    {        
        Console.WriteLine(((StandardString)message).data);
    }

    private static void ServiceCallHandler(Message message)
    {
        Console.WriteLine("ROS distro: " + ((RosApiGetParamResponse)message).value);
    }

    private static bool ServiceResponseHandler(Message arguments, out Message result)
    {
        result = new StandardServiceTriggerResponse(true, "service response message");
        return true;
    }

}