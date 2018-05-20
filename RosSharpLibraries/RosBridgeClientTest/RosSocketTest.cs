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

using NUnit.Framework;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClientTest
{
    [TestFixture]
    public class RosSocketTest
    {
        // on ROS system:
        // launch before starting:
        // roslaunch rosbridge_server rosbridge_websocket.launch
        // rostopic echo /publication_test
        // rostopic pub /subscription_test std_msgs/String "subscription test message data"

        // launch after starting:
        // rosservice call /service_response_test

        private static string Url = "ws://192.168.56.102:9090";
        private static RosSocket RosSocket;
        private ManualResetEvent OnMessageReceived = new ManualResetEvent(false);
        private ManualResetEvent OnServiceReceived = new ManualResetEvent(false);
        private ManualResetEvent OnServiceProvided = new ManualResetEvent(false);

        [SetUp]
        public void Setup()
        {
            RosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketSharpProtocol(Url));
        }

        [TearDown]
        public void TearDown()
        {
            RosSocket.Close();
        }

        [Test]
        public void PublicationTest()
        {
            string id = RosSocket.Advertise<StandardString>("/publication_test");
            StandardString message = new StandardString("publication test message data");
            RosSocket.Publish(id, message);
            RosSocket.Unadvertise(id);
            Thread.Sleep(100);
            Assert.IsTrue(true);
        }

        [Test]
        public void SubscriptionTest()
        {
            string id = RosSocket.Subscribe<StandardString>("/subscription_test", SubscriptionHandler);
            OnMessageReceived.WaitOne();
            OnMessageReceived.Reset();
            RosSocket.Unsubscribe(id);
            Thread.Sleep(100);
            Assert.IsTrue(true);
        }

        [Test]
        public void ServiceCallTest()
        {
            RosSocket.CallService<RosApiGetParamRequest, RosApiGetParamResponse>("/rosapi/get_param", ServiceCallHandler, new RosApiGetParamRequest("/rosdistro"));
            OnServiceReceived.WaitOne();
            OnServiceReceived.Reset();
            Assert.IsTrue(true);
        }

        [Test]
        public void ServiceResponseTest()
        {
            RosSocket.AdvertiseService<StandardServiceTriggerRequest, StandardServiceTriggerResponse>("/service_response_test", ServiceResponseHandler);
            OnServiceProvided.WaitOne();
            OnServiceProvided.Reset();
            Assert.IsTrue(true);
        }

        private void SubscriptionHandler(StandardString message)
        {
            OnMessageReceived.Set();
        }

        private void ServiceCallHandler(RosApiGetParamResponse message)
        {
            OnServiceReceived.Set();
        }

        private bool ServiceResponseHandler(StandardServiceTriggerRequest arguments, out StandardServiceTriggerResponse result)
        {
            result = new StandardServiceTriggerResponse(true, "service response message");
            OnServiceProvided.Set();
            return true;
        }
    }
}
