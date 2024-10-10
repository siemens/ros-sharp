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

using System;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using std_srvs = RosSharp.RosBridgeClient.MessageTypes.Std;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;


// commands on ROS system:
// launch before starting:
// ROS:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /pub_test
// rostopic pub /sub_test std_msgs/String "subscription test message data"
// launch after starting:
// rosservice call /service_response_test

// ROS2:
// ros2 launch rosbridge_server rosbridge_websocket.launch
// ros2 topic echo /pub_test
// ros2 topic pub -r 50 /sub_test std_msgs/String "data: subscription test message data"


namespace RosSharp.RosBridgeClientTest
{
    public class RosSocketConsole
    {
        static readonly string uri = "ws://localhost:9090";

        public static void Main(string[] args)
        {
            //RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketSharpProtocol(uri));
            RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

            // Publication:
            std_msgs.String message = new std_msgs.String
            {
                data = "publication test masdasdessage data"
            };

            string publication_id = rosSocket.Advertise<std_msgs.String>("pub_test");
            rosSocket.Publish(publication_id, message);

            // Subscription:
            string subscription_id = rosSocket.Subscribe<std_msgs.String>("/sub_test", SubscriptionHandler);

            // Service Call:
            rosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>("/rosapi/get_param", ServiceCallHandler, new rosapi.GetParamRequest("/rosdistro", "defaut_value")); // Just "default" for ROS1

            // Service Response:
            string service_id = rosSocket.AdvertiseService<std_srvs.TriggerRequest, std_srvs.TriggerResponse>("/service_response_test", ServiceResponseHandler);

            Console.WriteLine("Press any key to unsubscribe...");
            Console.ReadKey(true);
            rosSocket.Unadvertise(publication_id);
            rosSocket.Unsubscribe(subscription_id);
            rosSocket.UnadvertiseService(service_id);

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);
            rosSocket.Close();
        }
        private static void SubscriptionHandler(std_msgs.String message)
        {
            Console.WriteLine((message).data);
        }

        private static void ServiceCallHandler(rosapi.GetParamResponse message)
        {
            Console.WriteLine("ROS distro: " + message.value);
        }

        private static bool ServiceResponseHandler(std_srvs.TriggerRequest arguments, out std_srvs.TriggerResponse result)
        {
            result = new std_srvs.TriggerResponse(true, "service response message");
            return true;
        }
    }
}