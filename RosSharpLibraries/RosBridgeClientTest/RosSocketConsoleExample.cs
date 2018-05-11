using System;
using RosSharp.RosBridgeClient;
using Newtonsoft.Json.Linq;

// commands on ROS system:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /talker
// rostopic pub /listener std_msgs/String "World!"

public class RosSocketConsole
{
    private static string url = "ws://192.168.56.102:9090";

    public static void Main(string[] args)
    {
        RosSocket rosSocket = new RosSocket(url);

        // Publication:
        string publication_id = rosSocket.Advertise("/publication_test", "std_msgs/String");

        StandardString message = new StandardString();
        message.data = "Hello!";
        rosSocket.Publish(publication_id, message);

        // Subscription:
        string subscription_id = rosSocket.Subscribe("/subscription_test", "std_msgs/String", SubscriptionHandler);

        // Service Call:
        rosSocket.CallService("/rosapi/get_param", typeof(ParamValueString), ServiceCallHandler, new ParamName("/rosdistro"));

        // Service Response:
        rosSocket.AdvertiseService("/service_response_test", "std_srvs/Trigger", ServiceResponseHandler);

        Console.WriteLine("Press any key to close...");
        Console.ReadKey(true);
        rosSocket.Close();
    }
    private static void SubscriptionHandler(Message message)
    {
        
        StandardString standardString = (StandardString)message;
        Console.WriteLine(standardString.data);
    }

    private static void ServiceCallHandler(object message)
    {
        Console.WriteLine("ROS distro: " + ((ParamValueString)message).value);
    }

    private static bool ServiceResponseHandler(JObject arguments, out JObject result)
    {
        result = JObject.FromObject(new { success = true ,  message = "service response message" });
        return true;
    }

}