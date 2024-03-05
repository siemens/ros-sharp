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

// Added preprocessor directive flags for ROS2 support
// Siemens AG , 2024, Mehmet Emre Cakal (emre.cakal@siemens.com / m.emrecakal@gmail.com) 

using System;
using NUnit.Framework;
//using Newtonsoft.Json;
using System.Text.Json;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClientTest
{
    [TestFixture]
    public class RosCommunicationTests
    {
        [SetUp]
        public void Setup()
        {

        }

        [TearDown]
        public void TearDown()
        {

        }

        // Configure JsonSerializerOptions for indented formatting (system.text.json)
        private static readonly JsonSerializerOptions JsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true
            // Other options can be set here
        };

        [Test, Category("Offline")]
        public void PublicationTest()
        {
            Communication comm = new Publication<std_msgs.Time>("myid", "mytopic", new std_msgs.Time());
            //string json = JsonConvert.SerializeObject(comm); // for newtonsoft
            string json = JsonSerializer.Serialize(comm);      // for system.text.json

#if !ROS2
            Assert.AreEqual("{\"topic\":\"mytopic\",\"msg\":{\"secs\":0,\"nsecs\":0},\"op\":\"publish\",\"id\":\"myid\"}",
                           json);
#else
            Assert.AreEqual("{\"topic\":\"mytopic\",\"msg\":{\"sec\":0,\"nanosec\":0},\"op\":\"publish\",\"id\":\"myid\"}",
                           json);
#endif
            //Console.WriteLine("JSON:\n" + JsonConvert.SerializeObject(comm, Formatting.Indented) + "\n"); // for newtonsoft
            Console.WriteLine("JSON:\n" + JsonSerializer.Serialize(comm, JsonOptions) + "\n");              // for system.text.json
        }

        [Test, Category("Offline")]
        public void SubscriptionTest()
        {
            Communication comm = new Subscription("myid", "mytopic", "mytype");
            //string json = JsonConvert.SerializeObject(comm); // for newtonsoft
            string json = JsonSerializer.Serialize(comm);      // for system.text.json

            Assert.AreEqual("{\"topic\":\"mytopic\",\"type\":\"mytype\",\"throttle_rate\":0,\"queue_length\":1," +
                            "\"fragment_size\":2147483647,\"compression\":\"none\",\"op\":\"subscribe\",\"id\":\"myid\"}",
                            json);
            //Console.WriteLine("JSON:\n" + JsonConvert.SerializeObject(comm, Formatting.Indented) + "\n"); // for newtonsoft
            Console.WriteLine("JSON:\n" + JsonSerializer.Serialize(comm, JsonOptions) + "\n");              // for system.text.json
        }

        [Test, Category("Offline")]
        public void ServiceCallTest()
        {
            Communication comm = new ServiceCall<std_msgs.Time>("myid", "myservice", new std_msgs.Time());
            //string json = JsonConvert.SerializeObject(comm); // for newtonsoft
            string json = JsonSerializer.Serialize(comm);      // for system.text.json

#if !ROS2
            Assert.AreEqual("{\"service\":\"myservice\",\"args\":{\"secs\":0,\"nsecs\":0}," +
                            "\"fragment_size\":2147483647,\"compression\":\"none\",\"op\":\"call_service\",\"id\":\"myid\"}",
                            json);
#else       
            Assert.AreEqual("{\"service\":\"myservice\",\"args\":{\"sec\":0,\"nanosec\":0}," +
                            "\"fragment_size\":2147483647,\"compression\":\"none\",\"op\":\"call_service\",\"id\":\"myid\"}",
                            json);
#endif
            //Console.WriteLine("JSON:\n" + JsonConvert.SerializeObject(comm, Formatting.Indented) + "\n"); // for newtonsoft
            Console.WriteLine("JSON:\n" + JsonSerializer.Serialize(comm, JsonOptions) + "\n");              // for system.text.json
        }
    }
}
