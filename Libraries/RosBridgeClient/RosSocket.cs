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

// Adding BSON (de-)seriliazation option
// Shimadzu corp , 2019, Akira NODA (a-noda@shimadzu.co.jp / you.akira.noda@gmail.com)

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Text.Json;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient
{
    public class RosSocket
    {
        public IProtocol protocol;
        public enum SerializerEnum { JSON, BSON }

        private Dictionary<string, Publisher> Publishers = new Dictionary<string, Publisher>();
        private Dictionary<string, Subscriber> Subscribers = new Dictionary<string, Subscriber>();
        private Dictionary<string, ServiceProvider> ServiceProviders = new Dictionary<string, ServiceProvider>();
        private Dictionary<string, ServiceConsumer> ServiceConsumers = new Dictionary<string, ServiceConsumer>();
        private SerializerEnum Serializer;
        private object SubscriberLock = new object();

        public RosSocket(IProtocol protocol, SerializerEnum serializer = SerializerEnum.JSON)
        {
            this.protocol = protocol;
            this.Serializer = serializer;
            this.protocol.OnReceive += (sender, e) => Receive(sender, e);
            this.protocol.Connect();
        }

        public void Close(int millisecondsWait = 0)
        {
            bool isAnyCommunicatorActive = Publishers.Count > 0 || Subscribers.Count > 0 || ServiceProviders.Count > 0;

            while (Publishers.Count > 0)
                Unadvertise(Publishers.First().Key);

            while (Subscribers.Count > 0)
                Unsubscribe(Subscribers.First().Key);

            while (ServiceProviders.Count > 0)
                UnadvertiseService(ServiceProviders.First().Key);

            // Service consumers do not stay on. So nothing to unsubscribe/unadvertise

            if (isAnyCommunicatorActive)
            {
                Thread.Sleep(millisecondsWait);
            }

            protocol.Close();
        }

        #region Publishers

        public string Advertise<T>(string topic) where T : Message
        {
            string id = topic;
            if (Publishers.ContainsKey(id))
                Unadvertise(id);

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
            string id;
            lock (SubscriberLock)
            {
                id = GetUnusedCounterID(Subscribers, topic);
                Subscription subscription;
                Subscribers.Add(id, new Subscriber<T>(id, topic, subscriptionHandler, out subscription, throttle_rate, queue_length, fragment_size, compression));
                Send(subscription);
            }
            
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
            if (ServiceProviders.ContainsKey(id))
                UnadvertiseService(id);

            ServiceAdvertisement serviceAdvertisement;
            ServiceProviders.Add(id, new ServiceProvider<Tin, Tout>(service, serviceCallHandler, out serviceAdvertisement));
            Send(serviceAdvertisement);
            return id;
        }

        public void UnadvertiseService(string id)
        {
            Send(ServiceProviders[id].UnadvertiseService());
            ServiceProviders.Remove(id);
        }

        #endregion

        #region ServiceConsumers

        public string CallService<Tin, Tout>(string service, ServiceResponseHandler<Tout> serviceResponseHandler, Tin serviceArguments) where Tin : Message where Tout : Message
        {
            string id = GetUnusedCounterID(ServiceConsumers, service);
            Communication serviceCall;
            ServiceConsumers.Add(id, new ServiceConsumer<Tin, Tout>(id, service, serviceResponseHandler, out serviceCall, serviceArguments));
            Send(serviceCall);
            return id;
        }

        #endregion

        private void Send<T>(T communication) where T : Communication
        {
#if DEBUG
            Console.WriteLine("Sending:\n" + JsonSerializer.Serialize(communication, communication.GetType() ,new JsonSerializerOptions { WriteIndented = true }) + "\n");
#endif
            protocol.Send(Serialize<T>(communication));
            return;
        }

        private void Receive(object sender, EventArgs e)
        {
            JsonElement jsonObject = Deserialize(((MessageEventArgs)e).RawData);
#if DEBUG            
            Console.WriteLine("Received: \n" + JsonSerializer.Serialize(jsonObject, new JsonSerializerOptions { WriteIndented = true }) + "\n");
#endif
            switch (jsonObject.GetProperty("op").ToString())
            {
                case "publish":
                    {
                        var jsonObjectProperty = jsonObject.GetProperty("msg");
                        string topic = jsonObject.GetProperty("topic").ToString();
                        foreach (Subscriber subscriber in SubscribersOf(topic))
                            subscriber.Receive(jsonObjectProperty);
                        return;
                    }
                case "service_response":
                    {
                        var jsonObjectProperty = jsonObject.GetProperty("values");
                        string id = jsonObject.GetProperty("id").ToString();
                        ServiceConsumers[id].Consume(jsonObjectProperty);
                        return;
                    }
                case "call_service":
                    {
                        var jsonObjectProperty = jsonObject.GetProperty("args");
                        string id = jsonObject.GetProperty("id").ToString();
                        string service = jsonObject.GetProperty("service").ToString();
                        Send(ServiceProviders[service].Respond(id, jsonObjectProperty));
                        return;
                    }
            }
        }

        private List<Subscriber> SubscribersOf(string topic)
        {
            return Subscribers.Where(pair => pair.Key.StartsWith(topic + ":")).Select(pair => pair.Value).ToList();
        }

        private byte[] Serialize<T>(T obj)
        {
            switch (Serializer)
            {
                case SerializerEnum.JSON:
                    string json = JsonSerializer.Serialize(obj, obj.GetType());
                    return Encoding.ASCII.GetBytes(json);
                //case SerializerEnum.BSON:
                //    System.IO.MemoryStream ms = new System.IO.MemoryStream();
                //    Newtonsoft.Json.Bson.BsonDataWriter writer = new Newtonsoft.Json.Bson.BsonDataWriter(ms);
                //    Newtonsoft.Json.JsonSerializer serializer = new Newtonsoft.Json.JsonSerializer();
                //    serializer.Serialize(writer, obj);
                //    return ms.ToArray();
                default:
                    throw new ArgumentException("Invalid Serializer");
            }
        }

        private JsonElement Deserialize(byte[] buffer)
        {
            switch (Serializer)
            {
                case SerializerEnum.JSON:
                    string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
                    return JsonDocument.Parse(ascii).RootElement;
                //case SerializerEnum.BSON:
                //    System.IO.MemoryStream ms = new System.IO.MemoryStream(buffer);
                //    Newtonsoft.Json.Bson.BsonDataReader reader = new Newtonsoft.Json.Bson.BsonDataReader(ms);
                //    return new Newtonsoft.Json.JsonSerializer().Deserialize<T>(reader);
                default:
                    throw new ArgumentException("Invalid Serializer");
            }
        }

        private static string GetUnusedCounterID<T>(Dictionary<string, T> dictionary, string name)
        {
            int I = 0;
            string id;
            do
                id = name + ":" + I++;
            while (dictionary.ContainsKey(id));
            return id;
        }
    }
}
