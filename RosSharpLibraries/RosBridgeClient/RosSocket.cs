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
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Text;

using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClient
{    
    public class RosSocket
    {

        // TODO:

       // implement message type selection as generics
       // implement thread for service response
       // remove messagetypes class

        // split message file into individual classes
        // split operator file into individual classes
        

        private IProtocol Protocol;

        private Dictionary<string, Publisher> Publishers = new Dictionary<string, Publisher>();
        private Dictionary<string, Subscriber> Subscribers = new Dictionary<string, Subscriber>();
        private Dictionary<string, ServiceResponder> ServiceResponders = new Dictionary<string, ServiceResponder>();
        private Dictionary<string, ServiceCaller> ServiceCallers = new Dictionary<string, ServiceCaller>();

        public RosSocket(string url)
        {
            Protocol = new WebsocketProtocol(url);
            Protocol.OnMessage += (sender, e) => receivedOperation(sender, e);
            Protocol.Connect();
        }

        public void Close()
        {
            while (Publishers.Count > 0)
                Unadvertise(Publishers.First().Key);

            while (Subscribers.Count > 0)
                Unsubscribe(Subscribers.First().Key);

            while (ServiceResponders.Count > 0)
                UnadvertiseService(ServiceResponders.First().Key);

            Protocol.Close();
        }
        
        public string Advertise(string topic, string type)
        {
            string id = topic;
            Publishers.Add(id, new Publisher(topic));

            sendOperation(new Adverisement(id, topic, type));
            return id;
        }
        
        public void Publish(string id, Message message)
        {
            Publisher publisher;
            if (Publishers.TryGetValue(id, out publisher))
                sendOperation(new Publication(id, publisher.Topic, JObject.FromObject(message)));
        }

        public void Unadvertise(string id)
        {
            sendOperation(new Unadverisement(id, Publishers[id].Topic));
            Publishers.Remove(id);
        }

        public string AdvertiseService(string service, Type argumentType, ServiceCallHandler serviceCallHandler)
        {
            string id = service;
            ServiceResponders.Add(id, new ServiceResponder(service, argumentType, serviceCallHandler));

            sendOperation(new ServiceAdvertisement(id, service, MessageTypes.Dictionary[argumentType]));
            return id;
        }

        public void UnadvertiseService(string id)
        {
            sendOperation(new ServiceUnadvertisement(id, ServiceResponders[id].Service));
            ServiceResponders.Remove(id);
        }

        public string Subscribe(string topic, Type messageType, MessageHandler messageHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            string id = topic;
            Subscribers.Add(topic, new Subscriber(topic, messageType, messageHandler));
            sendOperation(new Subscription(id, topic, MessageTypes.Dictionary[messageType], throttle_rate, queue_length, fragment_size, compression));
            return id;
        }

        public void Unsubscribe(string id)
        {
            sendOperation(new Unsubscription(id, Subscribers[id].Topic));
            Subscribers.Remove(id);
        }

        public string CallService(string service, Type objectType, ServiceHandler serviceHandler, Message serviceArguments = null)
        {
            string id = service;
            ServiceCallers.Add(id, new ServiceCaller(service, objectType, serviceHandler));

            sendOperation(new ServiceCall(id, service, JObject.FromObject(serviceArguments)));
            return id;
        }

        private void receivedOperation(object sender, EventArgs e)
        {
            
            JObject jObject = Deserialize(((MessageEventArgs)e).RawData);

#if DEBUG            
            Console.WriteLine("Received:\n" + JsonConvert.SerializeObject(jObject, Formatting.Indented) + "\n");
#endif
            string id = jObject.GetValue("id")?.ToString();

            switch (jObject.GetValue("op").ToString())
            {
                case "publish":
                    {                        
                        receivedPublish(jObject.ToObject<Publication>());
                        return;
                    }
                case "service_response":
                    {
                        receivedServiceResponse(jObject.ToObject<ServiceResponse>());
                        return;
                    }
                case "call_service":
                    {
                        receivedServiceCall(jObject.ToObject<ServiceCall>());
                        return;
                    }
            }
        }
        private void receivedPublish(Publication publication)
        {
            Subscriber subscriber;
            if (!Subscribers.TryGetValue(publication.topic, out subscriber))
                return;

            subscriber.MessageHandler?.Invoke((Message)publication.msg.ToObject(subscriber.MessageType));
        }
        private void receivedServiceResponse(ServiceResponse serviceResponse)
        {
            ServiceCaller serviceCaller;
            if (!ServiceCallers.TryGetValue(serviceResponse.id, out serviceCaller))
                return;

            serviceCaller.ServiceHandler?.Invoke((Message)serviceResponse.values.ToObject(serviceCaller.ResponseType));
        }

        private void receivedServiceCall(ServiceCall serviceCall)
        {
            ServiceResponder serviceResponder;
            if (!ServiceResponders.TryGetValue(serviceCall.service, out serviceResponder))
                return;

            Message result;
            bool isSuccess = serviceResponder.ServiceCallHandler.Invoke((Message)serviceCall.args.ToObject(serviceResponder.ArgumentType), out result);


            // data is lost when parsing as message

            // implement thread here
            sendOperation(
                new ServiceResponse(
                serviceCall.id,
                serviceCall.service,
                JObject.FromObject(result),
                isSuccess));
        }

        private void sendOperation(Operation operation)
        {
#if DEBUG
            Console.WriteLine("Sending:\n" +  JsonConvert.SerializeObject(operation, Formatting.Indented) + "\n");           
#endif
            Protocol.SendAsync(Serialize(operation), null);
        }

        public static byte[] Serialize(object obj)
        {
            string json = JsonConvert.SerializeObject(obj);
            return Encoding.ASCII.GetBytes(json);
        }

        public static JObject Deserialize(byte[] buffer)
        {
            string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
            return JsonConvert.DeserializeObject<JObject>(ascii);
        }
    }
}
