/*
© Siemens AG, 2017-2018
Author: Manuel Stahl (manuel.stahl@awesome-technologies.de)

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
using NUnit.Framework;
using Newtonsoft.Json.Linq;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using System.Diagnostics;
using RosSharp.RosBridgeClient.Serializers;

namespace RosSharp.RosBridgeClientTest
{

    [TestFixture(typeof(JsonNetSerializer))]
    [TestFixture(typeof(BsonNetSerializer))]
    [TestFixture(typeof(Utf8JsonSerializer))]
    public class RosCommunicationTests<T> where T : ISerializer, new()
    {
        T serializer = new T();

        [SetUp]
        public void Setup()
        {

        }

        [TearDown]
        public void TearDown()
        {

        }
        
        [Test, Category("Offline")]
        public void PublicationTest()
        {
            Publication<std_msgs.Time> comm = new Publication<std_msgs.Time>("myid", "mytopic", new std_msgs.Time());
            byte[] bytes = serializer.Serialize(comm);
            Console.WriteLine("JSON:\n" + serializer.GetJsonString(bytes) + "\n");
            JObject actual = JObject.Parse(serializer.GetJsonString(bytes));
            JObject expected = JObject.Parse("{\"topic\":\"mytopic\",\"msg\":{\"secs\":0,\"nsecs\":0},\"op\":\"publish\",\"id\":\"myid\"}");
            Assert.IsTrue(JToken.DeepEquals(expected, actual));
        }

        [Test, Category("Offline")]
        public void SubscriptionTest()
        {
            Subscription comm = new Subscription("myid", "mytopic", "mytype");
            byte[] bytes = serializer.Serialize(comm);
            Console.WriteLine("JSON:\n" + serializer.GetJsonString(bytes) + "\n");
            JObject actual = JObject.Parse(serializer.GetJsonString(bytes));
            JObject expected = JObject.Parse("{\"topic\":\"mytopic\",\"type\":\"mytype\",\"throttle_rate\":0,\"queue_length\":1," +
                "\"fragment_size\":2147483647,\"compression\":\"none\",\"op\":\"subscribe\",\"id\":\"myid\"}");
            Assert.IsTrue(JToken.DeepEquals(expected, actual));
        }

        [Test, Category("Offline")]
        public void ServiceCallTest()
        {
            ServiceCall<std_msgs.Time> comm = new ServiceCall<std_msgs.Time>("myid", "myservice", new std_msgs.Time());
            byte[] bytes = serializer.Serialize(comm);
            string s = serializer.GetJsonString(bytes);
            Console.WriteLine("JSON:\n" + s + "\n");
            JObject actual = JObject.Parse(s);
            JObject expected = JObject.Parse("{\"service\":\"myservice\",\"args\":{\"secs\":0,\"nsecs\":0}," +
                          "\"fragment_size\":2147483647,\"compression\":\"none\",\"op\":\"call_service\",\"id\":\"myid\"}");
            Assert.IsTrue(JToken.DeepEquals(expected, actual));
        }

        [Test, Category("Offline")]
        public void ReceivedMessageTest()
        {
            std_msgs.Time time = new std_msgs.Time();
            Publication<std_msgs.Time> comm = new Publication<std_msgs.Time>("myid", "mytopic", time);
            byte[] bytes = serializer.Serialize(comm);
            Console.WriteLine("JSON:\n" + serializer.GetJsonString(bytes) + "\n");
            JObject actual = JObject.Parse(serializer.GetJsonString(bytes));
            JObject expected = JObject.Parse("{\"topic\":\"mytopic\",\"msg\":{\"secs\":0,\"nsecs\":0},\"op\":\"publish\",\"id\":\"myid\"}");
            Assert.IsTrue(JToken.DeepEquals(expected, actual));
            IReceivedMessage received = serializer.DeserializeReceived(bytes);
            Assert.IsTrue(received.Op == "publish");
            Assert.IsTrue(received.Id == "myid");
            Assert.IsTrue(received.Topic == "mytopic");
            Assert.IsTrue(received.Service == null);
            Assert.IsNull(received.GetValues<std_msgs.Time>());
            Assert.IsNull(received.GetArgs<std_msgs.Time>());
            std_msgs.Time receivedMsg = received.GetMessage<std_msgs.Time>();
            Assert.IsTrue(receivedMsg.secs == time.secs);
            Assert.IsTrue(receivedMsg.nsecs == time.nsecs);
        }
    }
}
