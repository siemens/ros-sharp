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
using RosSharp.RosBridgeClient.Serializers;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient
{
    public class RosSocket
    {
        public IProtocol protocol;
        public enum SerializerEnum { JSON, BSON, UTF8JSON }

        private Dictionary<string, Publisher> Publishers = new Dictionary<string, Publisher>();
        private Dictionary<string, Subscriber> Subscribers = new Dictionary<string, Subscriber>();
        private Dictionary<string, ServiceProvider> ServiceProviders = new Dictionary<string, ServiceProvider>();
        private Dictionary<string, ServiceConsumer> ServiceConsumers = new Dictionary<string, ServiceConsumer>();
        private ISerializer Serializer;
        private object SubscriberLock = new object();

        public RosSocket(IProtocol protocol, SerializerEnum serializer = SerializerEnum.JSON)
        {
            this.protocol = protocol;

            switch (serializer)
            {
                case SerializerEnum.JSON:
                    this.Serializer = new JsonNetSerializer();
                    break;
                case SerializerEnum.BSON:
                    this.Serializer = new BsonNetSerializer();
                    break;
                case SerializerEnum.UTF8JSON:
                    this.Serializer = new Utf8JsonSerializer();
                    break;
                default:
                    throw new ArgumentException("Serializer exception");
            }

            this.protocol.OnReceive += Receive;
            this.protocol.OnSent += OnSent; 
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

        public void Publish<T>(string id, T message) where T : Message
        {
            Publisher pub = Publishers[id];
            Communication comm = pub.Publish(message);
            Send((Publication<T>)comm);
            pub.Return(comm);
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
            Send((ServiceCall<Tin>) serviceCall);
            return id;
        }

        #endregion

        private void Send<T>(T communication) where T : Communication
        {
#if DEBUG
            Console.WriteLine("Sending:\n" + Serializer.GetJsonString(Serializer.Serialize(communication)) + "\n");
#endif
            if (protocol is WebSocketNetChannelsProtocol)
            {
                // Get the deserialized bytes that are stored in some internal buffer and copy to our own
                ArraySegment<byte> buf = Serializer.SerializeUnsafe(communication);
                byte[] msg = System.Buffers.ArrayPool<byte>.Shared.Rent(buf.Count);
                Buffer.BlockCopy(buf.Array, buf.Offset, msg, 0, buf.Count);
                protocol.Send(new ArraySegment<byte>(msg, 0, buf.Count));
            }
            else
                protocol.Send(Serializer.Serialize<T>(communication));

            return;
        }

        private void OnSent(object caller, EventArgs e)
        {
            if (protocol is WebSocketNetChannelsProtocol)
            {
                System.Buffers.ArrayPool<byte>.Shared.Return(((MessageEventArgs)e).RawData);
            }
        }

        private void Receive(object sender, EventArgs e)
        {
            IReceivedMessage commHandle = Serializer.DeserializeReceived(((MessageEventArgs)e).RawData);
#if DEBUG
            Console.WriteLine("Received:\n" + Serializer.GetJsonString(((MessageEventArgs)e).RawData) + "\n");
#endif
            switch (commHandle.Op)
            {
                case "publish":
                    {
                        foreach (Subscriber subscriber in SubscribersOf(commHandle.Topic))
                            subscriber.Receive(commHandle);
                        return;
                    }
                case "service_response":
                    {
                        ServiceConsumers[commHandle.Id].Consume(commHandle);
                        return;
                    }
                case "call_service":
                    {
                        Send(ServiceProviders[commHandle.Service].Respond(commHandle.Id, commHandle));
                        return;
                    }
            }
        }
        private List<Subscriber> SubscribersOf(string topic)
        {
            return Subscribers.Where(pair => pair.Key.StartsWith(topic + ":")).Select(pair => pair.Value).ToList();
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
