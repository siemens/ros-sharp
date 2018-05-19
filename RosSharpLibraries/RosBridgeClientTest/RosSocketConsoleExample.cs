using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages;

// commands on ROS system:
// launch before starting:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /publication_test
// rostopic pub /subscription_test std_msgs/String "subscription test message data"

// launch after starting:
// rosservice call /service_response_test

namespace RosSharp.RosBridgeClientTest
{
    public class RosSocketConsole
    {
        public static void Main(string[] args)
        {
            RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebsocketProtocol("ws://192.168.56.102:9090"));

            // Publication:
            string publication_id = rosSocket.Advertise<StandardString>("/publication_test");
            StandardString message = new StandardString("publication test message data");
            rosSocket.Publish(publication_id, message);

            // Subscription:
            string subscription_id = rosSocket.Subscribe<StandardString>("/subscription_test", SubscriptionHandler);

            // Service Call:
            rosSocket.CallService<RosApiGetParamRequest, RosApiGetParamResponse>("/rosapi/get_param", ServiceCallHandler, new RosApiGetParamRequest("/rosdistro"));

            // Service Response:
            rosSocket.AdvertiseService<StandardServiceTriggerRequest, StandardServiceTriggerResponse>("/service_response_test", ServiceResponseHandler);

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);
            rosSocket.Close();
        }
        private static void SubscriptionHandler(StandardString message)
        {
            Console.WriteLine((message).data);
        }

        private static void ServiceCallHandler(RosApiGetParamResponse message)
        {
            Console.WriteLine("ROS distro: " + message.value);
        }

        private static bool ServiceResponseHandler(StandardServiceTriggerRequest arguments, out StandardServiceTriggerResponse result)
        {
            result = new StandardServiceTriggerResponse(true, "service response message");
            return true;
        }
    }
}