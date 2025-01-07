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
using System.Threading;
using System.Threading.Tasks;
using System.Text;
using System.Collections.Generic;
using System.Reflection;


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

using RosSharp.RosBridgeClient.MessageTypes.ActionTutorialsInterfaces;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClientTest
{
    public class RosSocketConsole
    {
        static readonly string uri = "ws://localhost:9090";

        public static void Main(string[] args)
        {
            //RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketSharpProtocol(uri));
            RosSocket rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(uri));

            // Publication
            SimpleMessagePub(rosSocket, "/pub_test");

            // Subscription:
            SimpleMessageSub(rosSocket, "/sub_test");

            // Thread Safety Test:
            SimulateParallelMessageReception(rosSocket, 8);

            // Service Call:
            //rosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>("/rosapi/get_param", ServiceCallHandler, new rosapi.GetParamRequest("/rosdistro", "default")); //  ROS1
            rosSocket.CallService<rosapi.GetROSVersionRequest, rosapi.GetROSVersionResponse>("/rosapi/get_ros_version", ServiceCallHandlerROS2, new rosapi.GetROSVersionRequest());

            // Service Response:
            string service_id = rosSocket.AdvertiseService<std_srvs.TriggerRequest, std_srvs.TriggerResponse>("/service_response_test", ServiceResponseHandler);
            Console.WriteLine("Service id: " + service_id);
            Console.WriteLine("Press any key to close unadvertise service...");
            Console.ReadKey(true);
            rosSocket.UnadvertiseService(service_id);

            Console.WriteLine("Press any key to close RosSocket...");
            Console.ReadKey(true);
            rosSocket.Close();
        }
        private static void actionGoalHandler(std_msgs.String message)
        {
            Console.WriteLine("Is this working?");
            Console.WriteLine((message).data);
        }
        private static void SubscriptionHandler(std_msgs.String message)
        {
            Console.WriteLine("processing message: " + (message).data);
            Thread.Sleep(100); // simulate some work
            Console.WriteLine("done processing message: " + (message).data);
        }

        private static void ServiceCallHandler(rosapi.GetParamResponse message)
        {
            Console.WriteLine("Response value: " + message.value);
        }

        private static void ServiceCallHandlerROS2(rosapi.GetROSVersionResponse message)
        {
            Console.WriteLine("Response value: " + message.distro);
        }

        private static bool ServiceResponseHandler(std_srvs.TriggerRequest arguments, out std_srvs.TriggerResponse result)
        {
            result = new std_srvs.TriggerResponse(true, "service response message");
            return true;
        }

        // Simple message pub
        private static void SimpleMessagePub(RosSocket rosSocket, String topic)
        {
            std_msgs.String message = new std_msgs.String
            {
                data = "single pub test msg"
            };

            string publication_id = rosSocket.Advertise<std_msgs.String>(topic);
            rosSocket.Publish(publication_id, message);

            Console.WriteLine("Press any key to unadvertise...");
            Console.ReadKey(true);

            rosSocket.Unadvertise(publication_id);
        }

        // Simple message sub
        private static void SimpleMessageSub(RosSocket rosSocket, String topic)
        {
            string subscription_id = rosSocket.Subscribe<std_msgs.String>(topic, SubscriptionHandler, ensureThreadSafety: false);

            Console.WriteLine("Press any key to unsubscribe...");
            Console.ReadKey(true);

            rosSocket.Unsubscribe(subscription_id);
        }

        // Simulate receiving messages from multiple threads to test thread safety
        private static void SimulateParallelMessageReception(RosSocket rosSocket, int numberOfMessages)
        {
            string subscription_id = rosSocket.Subscribe<std_msgs.String>("/thread_test", SubscriptionHandler, ensureThreadSafety: true);
            var rosSocketType = rosSocket.GetType();
            var subscribersField = rosSocketType.GetField("Subscribers", BindingFlags.NonPublic | BindingFlags.Instance);
            var subscribers = (Dictionary<string, Subscriber>)subscribersField.GetValue(rosSocket);
            var subscriber = subscribers[subscription_id];

            Parallel.For(0, numberOfMessages, i =>
            {
                // Fake message data
                var message = new std_msgs.String { data = $"message {i}" };
                
                byte[] serializedBytes = rosSocket.Serializer.Serialize(message);
                string serializedMessage = Encoding.UTF8.GetString(serializedBytes);
                var deserializedMessage = rosSocket.Serializer.Deserialize<std_msgs.String>(serializedMessage);

                Console.WriteLine($"Processing thread: {i}");

                subscriber.Receive(serializedMessage, rosSocket.Serializer);
            });

            rosSocket.Unsubscribe(subscription_id);
        }
    }
}
