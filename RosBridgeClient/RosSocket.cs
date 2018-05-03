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
using System.Linq;
using System.Collections.Generic;
using WebSocketSharp;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Text;

namespace RosSharp.RosBridgeClient
{
    public class RosSocket
    {
        #region Public
        public RosSocket(string url)
        {
            webSocket = new WebSocket(url);
            webSocket.OnMessage += (sender, e) => receivedOperation((WebSocket)sender, e);
            webSocket.Connect();
        }

        public void Close()
        {
            while (publishers.Count > 0)
                Unadvertise(publishers.First().Key);

            while (subscribers.Count > 0)
                Unsubscribe(subscribers.First().Key);

            while (serviceProviders.Count > 0)
                UnadvertiseService(serviceProviders.First().Key);

            webSocket.Close();
        }

        public delegate void ServiceHandler(object obj);
        public delegate void MessageHandler(Message message);
        public delegate bool ServiceCallHandler(JObject arguments, out JObject reult);

        public string Advertise(string topic, string type)
        {
            string id = generateId();
            publishers.Add(id, new Publisher(topic));

            sendOperation(new Adverisement(id, topic, type));
            return id;
        }

        public string AdvertiseService(string service, string argumentType, ServiceCallHandler serviceCallHandler)
        {
            string id = generateId();
            serviceProviders.Add(id, new ServiceProvider(service, argumentType, serviceCallHandler));

            sendOperation(new ServiceAdvertisement(id, service, argumentType));
            return id;
        }

        public void Publish(string id, Message msg)
        {
            Publisher publisher;
            if (publishers.TryGetValue(id, out publisher))
                sendOperation(new Publication(id, publisher.Topic, msg));
        }

        public void Unadvertise(string id)
        {
            sendOperation(new Unadverisement(id, publishers[id].Topic));
            publishers.Remove(id);
        }

        public void UnadvertiseService(string id)
        {
            sendOperation(new ServiceUnadvertisement(id, serviceProviders[id].Service));
            serviceProviders.Remove(id);
        }

        public string Subscribe(string topic, string rosMessageType, MessageHandler messageHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            Type messageType = MessageTypes.MessageType(rosMessageType);
            if (messageType == null)
                return null;

            string id = generateId();
            subscribers.Add(id, new Subscriber(topic, messageType, messageHandler));
            sendOperation(new Subscription(id, topic, rosMessageType, throttle_rate, queue_length, fragment_size, compression));
            return id;
        }

        public int Subscribe(string topic, Type messageType, MessageHandler messageHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            string rosMessageType = MessageTypes.RosMessageType(messageType);
            if (rosMessageType == null)
                return 0;

            return Subscribe(topic, messageType, messageHandler, throttle_rate, queue_length, fragment_size, compression);
        }

        public void Unsubscribe(string id)
        {
            sendOperation(new Unsubscription(id, subscribers[id].topic));
            subscribers.Remove(id);
        }

        public string CallService(string service, Type objectType, ServiceHandler serviceHandler, object args = null)
        {
            string id = generateId();
            serviceCallers.Add(id, new ServiceCaller(service, objectType, serviceHandler));

            sendOperation(new ServiceCall(id, service, args));
            return id;
        }
        
        #endregion

        #region Private

        internal struct Publisher
        {
            internal string Topic;
            internal Publisher(string topic)
            {
                Topic = topic;
            }
        }

        internal struct ServiceProvider
        {
            internal string Service;
            internal string ArgumentType;
            internal ServiceCallHandler ServiceCallHandler;
            internal ServiceProvider(string service, string argumentType, ServiceCallHandler serviceCallHandler)
            {
                Service = service;
                ArgumentType = argumentType;
                ServiceCallHandler = serviceCallHandler;
            }
        }

        internal struct Subscriber
        {
            internal string topic;
            internal Type messageType;
            internal MessageHandler messageHandler;
            internal Subscriber(string Topic, Type MessageType, MessageHandler MessageHandler)
            {
                topic = Topic;
                messageType = MessageType;
                messageHandler = MessageHandler;
            }
        }
        internal struct ServiceCaller
        {
            internal string Service;
            internal Type ResponseType;
            internal ServiceHandler ServiceHandler;
            internal ServiceCaller(string service, Type responseType, ServiceHandler serviceHandler)
            {
                Service = service;
                ResponseType = responseType;
                ServiceHandler = serviceHandler;
            }
        }

        private WebSocket webSocket;
        private Dictionary<string, ServiceProvider> serviceProviders = new Dictionary<string, ServiceProvider>();
        private Dictionary<string, Publisher> publishers = new Dictionary<string, Publisher>();
        private Dictionary<string, Subscriber> subscribers = new Dictionary<string, Subscriber>();
        private Dictionary<string, ServiceCaller> serviceCallers = new Dictionary<string, ServiceCaller>();

        private void receivedOperation(object sender, MessageEventArgs e)
        {
            JObject operation = Deserialize(e.RawData);

#if DEBUG
            Console.WriteLine("Received " + operation.GetOperation());
            Console.WriteLine(JsonConvert.SerializeObject(operation, Formatting.Indented));
#endif

            switch (operation.GetOperation())
            {
                case "publish":
                    {
                        receivedPublish(operation, e.RawData);
                        return;
                    }
                case "service_response":
                    {
                        receivedServiceResponse(operation, e.RawData);
                        return;
                    }
                case "call_service":
                    {
                        receivedServiceCall(operation, e.RawData);
                        return;
                    }
            }
        }

        private void receivedServiceCall(JObject serviceCall, byte[] rawData)
        {
            JObject result;
            ServiceProvider serviceProvider = serviceProviders.Values.FirstOrDefault(x => x.Service.Equals(serviceCall.GetService()));

            bool isSuccess = serviceProvider.ServiceCallHandler.Invoke(serviceCall.GetArguments(), out result);

            ServiceResponse serviceResponse = new ServiceResponse(
                serviceCall.GetServiceId(),
                serviceCall.GetService(),
                result,
                isSuccess);
            sendOperation(serviceResponse);
        }



        private void receivedServiceResponse(JObject serviceResponse, byte[] rawData)
        {
            ServiceCaller serviceCaller;
            bool foundById = serviceCallers.TryGetValue(serviceResponse.GetServiceId(), out serviceCaller);

            if (!foundById)
                serviceCaller = serviceCallers.Values.FirstOrDefault(x => x.Service.Equals(serviceResponse.GetService()));

            JObject jObject = serviceResponse.GetValues();
            Type type = serviceCaller.ResponseType;
            if (type != null)
                serviceCaller.ServiceHandler?.Invoke(jObject.ToObject(type));
            else
                serviceCaller.ServiceHandler?.Invoke(jObject);
        }

        private void receivedPublish(JObject publication, byte[] rawData)
        {
            Subscriber subscriber;

            bool foundById = subscribers.TryGetValue(publication.GetServiceId(), out subscriber);

            if (!foundById)
                subscriber = subscribers.Values.FirstOrDefault(x => x.topic.Equals(publication.GetTopic()));

            subscriber.messageHandler?.Invoke((Message)publication.GetMessage().ToObject(subscriber.messageType));
        }

        private void sendOperation(Operation operation)
        {
#if DEBUG
            Console.WriteLine(JsonConvert.SerializeObject(operation, Formatting.Indented));
#endif
            webSocket.SendAsync(Serialize(operation), null);
        }
        public static byte[] Serialize(object obj)
        {
            string json = JsonConvert.SerializeObject(obj);
            byte[] buffer = Encoding.ASCII.GetBytes(json);
            int I = json.Length;
            int J = buffer.Length;
            return buffer;
        }

        public static JObject Deserialize(byte[] buffer)
        {
            string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
            int I = ascii.Length;
            int J = buffer.Length;
            return JsonConvert.DeserializeObject<JObject>(ascii);
        }

        private static string generateId()
        {
            return Guid.NewGuid().GetHashCode().ToString();
        }
        #endregion       
    }
}
