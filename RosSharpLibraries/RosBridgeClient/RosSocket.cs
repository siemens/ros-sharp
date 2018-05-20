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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.Messages;
using RosSharp.RosBridgeClient.Processing;

namespace RosSharp.RosBridgeClient
{
    public class RosSocket
    {
        // TODO:

        // combine service request and response messages in one service class?

        // split message file into individual classes
        // split operator file into individual classes

        private IProtocol Protocol;

        private Dictionary<string, Publisher> Publishers = new Dictionary<string, Publisher>();
        private Dictionary<string, Subscriber> Subscribers = new Dictionary<string, Subscriber>();
        private Dictionary<string, ServiceProvider> ServiceProvider = new Dictionary<string, ServiceProvider>();
        private Dictionary<string, ServiceConsumer> ServiceConsumers = new Dictionary<string, ServiceConsumer>();

        public RosSocket(IProtocol protocol)
        {
            Protocol = protocol;
            Protocol.OnReceive += (sender, e) => Receive(sender, e);
            Protocol.Connect();
        }

        public void Close()
        {
            while (Publishers.Count > 0)
                Unadvertise(Publishers.First().Key);

            while (Subscribers.Count > 0)
                Unsubscribe(Subscribers.First().Key);

            while (ServiceProvider.Count > 0)
                UnadvertiseService(ServiceProvider.First().Key);

            Protocol.Close();
        }

        #region Publishers

        public string Advertise<T>(string topic) where T : Message
        {
            // todo delete previous entry if topic already exists

            string id = topic;
            Publishers.Add(id, new Publisher<T>(id, topic, out Advertisement advertisement));
            Send(advertisement);
            return id;
        }

        public void Publish(string id, Message message)
        {
            Send(Publishers[id].Publish(message));
        }

        public void Unadvertise(string id)
        {
            Send(Publishers[id].Unadvertise());
            Publishers.Remove(id);
        }

        #endregion

        #region Subscribers

        public string Subscribe<T>(string topic, SubscriptionHandler<T> subscriptionHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") where T : Message
        {
            string id = topic;
            Subscribers.Add(id, new Subscriber<T>(id, topic, subscriptionHandler, out Subscription subscription, throttle_rate, queue_length, fragment_size, compression));
            Send(subscription);
            return id;
        }

        public void Unsubscribe(string id)
        {
            Send(Subscribers[id].Unsubscribe());
            Subscribers.Remove(id);
        }
        #endregion

        #region ServiceProviders

        public string AdvertiseService<Tin, Tout>(string service, ServiceCallHandler<Tin, Tout> serviceCallHandler) where Tin : Message where Tout : Message
        {
            string id = service;
            ServiceProvider.Add(id, new ServiceProvider<Tin, Tout>(service, serviceCallHandler, out ServiceAdvertisement serviceAdvertisement));
            Send(serviceAdvertisement);
            return id;
        }

        public void UnadvertiseService(string id)
        {
            Send(ServiceProvider[id].UnadvertiseService());
            ServiceProvider.Remove(id);
        }

        #endregion

        #region ServiceConsumers

        public string CallService<Tin, Tout>(string service, ServiceResponseHandler<Tout> serviceResponseHandler, Tin serviceArguments = null) where Tin : Message where Tout : Message
        {
            string id = service;
            ServiceConsumers.Add(id, new ServiceConsumer<Tin, Tout>(id, service, serviceResponseHandler, out Communication serviceCall, serviceArguments = null));
            Send(serviceCall);
            return id;
        }

        #endregion

        private void Send(Communication operation)
        {
#if DEBUG
            Console.WriteLine("Sending:\n" + JsonConvert.SerializeObject(operation, Formatting.Indented) + "\n");
#endif
            Protocol.Send(Serialize(operation));
            return;
        }

        private void Receive(object sender, EventArgs e)
        {
            JObject jObject = Deserialize<JObject>(((MessageEventArgs)e).RawData);
#if DEBUG            
            Console.WriteLine("Received:\n" + JsonConvert.SerializeObject(jObject, Formatting.Indented) + "\n");
#endif
            switch (jObject.GetValue("op").ToString())
            {
                case "publish":
                    {
                        string id = jObject.GetValue("topic").ToString();
                        Subscribers[id].Receive(jObject.GetValue("msg").ToObject<Message>());
                        return;
                    }
                case "service_response":
                    {
                        string id = jObject.GetValue("id").ToString();
                        ServiceConsumers[id].Consume(jObject.GetValue("values").ToObject<Message>());
                        return;
                    }
                case "call_service":
                    {
                        string id = jObject.GetValue("id").ToString();
                        ServiceProvider[id].Respond(id, jObject.GetValue("args").ToObject<Message>());
                        return;
                    }
            }
        }

        private static byte[] Serialize<T>(T obj)
        {
            string json = JsonConvert.SerializeObject(obj);
            return Encoding.ASCII.GetBytes(json);
        }

        private static T Deserialize<T>(byte[] buffer)
        {
            string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
            return JsonConvert.DeserializeObject<T>(ascii);
        }

    }
}
