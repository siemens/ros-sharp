/*
© Siemens AG, 2017-2018
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

using System.Threading;
using NUnit.Framework;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;
using std_srvs = RosSharp.RosBridgeClient.Services.Standard;
using rosapi = RosSharp.RosBridgeClient.Services.RosApi;

namespace RosSharp.RosBridgeClientTest
{
    [TestFixture]
    public class RosSocketTests
    {
        // on ROS system:
        // launch before starting:
        // roslaunch rosbridge_server rosbridge_websocket.launch
        // rostopic echo /publication_test
        // rostopic pub /subscription_test std_msgs/String "subscription test message data"

        // launch after starting:
        // rosservice call /service_response_test

        private static readonly string Uri = "ws://192.168.56.101:9090";
        private static RosSocket RosSocket;
        private ManualResetEvent OnMessageReceived = new ManualResetEvent(false);
        private ManualResetEvent OnServiceReceived = new ManualResetEvent(false);
        private ManualResetEvent OnServiceProvided = new ManualResetEvent(false);

        [SetUp]
        public void Setup()
        {
            RosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(Uri));
        }

        [TearDown]
        public void TearDown()
        {
            RosSocket.Close();
        }
        
        [Test]
        public void PublicationTest()
        {
            string id = RosSocket.Advertise<std_msgs.String>("/publication_test");
            std_msgs.String message = new std_msgs.String
            {
                data = "publication test message data"
            };
            RosSocket.Publish(id, message);
            RosSocket.Unadvertise(id);
            Thread.Sleep(100);
            Assert.IsTrue(true);
        }

        [Test]
        public void SubscriptionTest()
        {
            string id = RosSocket.Subscribe<std_msgs.String>("/subscription_test", SubscriptionHandler);
            OnMessageReceived.WaitOne();
            OnMessageReceived.Reset();
            RosSocket.Unsubscribe(id);
            Thread.Sleep(100);
            Assert.IsTrue(true);
        }

        [Test]
        public void ServiceCallTest()
        {
            RosSocket.CallService<rosapi.GetParamRequest, rosapi.GetParamResponse>("/rosapi/get_param", ServiceCallHandler, new rosapi.GetParamRequest("/rosdistro", "default"));
            OnServiceReceived.WaitOne();
            OnServiceReceived.Reset();
            Assert.IsTrue(true);
        }

        [Test]
        public void ServiceResponseTest()
        {
            string id = RosSocket.AdvertiseService<std_srvs.TriggerRequest, std_srvs.TriggerResponse>("/service_response_test", ServiceResponseHandler);
            OnServiceProvided.WaitOne();
            OnServiceProvided.Reset();
            RosSocket.UnadvertiseService(id);
            Assert.IsTrue(true);
        }

        private void SubscriptionHandler(std_msgs.String message)
        {
            OnMessageReceived.Set();
        }

        private void ServiceCallHandler(rosapi.GetParamResponse message)
        {
            OnServiceReceived.Set();
        }

        private bool ServiceResponseHandler(std_srvs.TriggerRequest arguments, out std_srvs.TriggerResponse result)
        {
            result = new std_srvs.TriggerResponse(true, "service response message");
            OnServiceProvided.Set();
            return true;
        }
    }
}
