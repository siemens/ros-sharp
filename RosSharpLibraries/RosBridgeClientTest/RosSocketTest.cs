using NUnit.Framework;
using System.Threading;
using RosSharp.RosBridgeClient;
using Newtonsoft.Json.Linq;

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
            RosSocket = new RosSocket(Url);
        }

        [TearDown]
        public void TearDown()
        {
            RosSocket.Close();
        }

        [Test]
        public void PublicationTest()
        {
            string id = RosSocket.Advertise("/publication_test", "std_msgs/String");
            StandardString message = new StandardString();
            message.data = "publication test message data";
            RosSocket.Publish(id, message);
            RosSocket.Unadvertise(id);
            Thread.Sleep(100);
            Assert.IsTrue(true);
        }

        [Test]
        public void SubscriptionTest()
        {
            string id = RosSocket.Subscribe("/subscription_test", "std_msgs/String", SubscriptionHandler);
            OnMessageReceived.WaitOne();
            OnMessageReceived.Reset();
            RosSocket.Unsubscribe(id);
            Thread.Sleep(100);
            Assert.IsTrue(true);
        }

        [Test]
        public void ServiceCallTest()
        {
            RosSocket.CallService("/rosapi/get_param", typeof(StandardString), ServiceCallHandler, new ParamName("/rosdistro"));
            OnServiceReceived.WaitOne();
            OnServiceReceived.Reset();
            Assert.IsTrue(true);
        }

        [Test]
        public void ServiceResponseTest()
        {
            RosSocket.AdvertiseService("/service_response_test", "std_srvs/Trigger", ServiceResponseHandler);
            OnServiceProvided.WaitOne();
            OnServiceProvided.Reset();
            Assert.IsTrue(true);
        }

        private void SubscriptionHandler(Message message)
        {
            OnMessageReceived.Set();
        }

        private void ServiceCallHandler(object message)
        {
            OnServiceReceived.Set();
        }

        private bool ServiceResponseHandler(JObject arguments, out JObject result)
        {
            result = JObject.FromObject(new { success = true, message = "service response message" });
            OnServiceProvided.Set();
            return true;
        }


    }
}
