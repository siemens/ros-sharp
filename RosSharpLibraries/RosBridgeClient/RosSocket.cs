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

       // finalize generic messagew type implementation

       // implement thread for service response
       // remove messagetypes class

        // split message file into individual classes
        // split operator file into individual classes
        

        private IProtocol Protocol;

        private Dictionary<string, Publisher> Publishers = new Dictionary<string, Publisher>();
       private Dictionary<string, Operator> Subscribers = new Dictionary<string, Operator>();
        private Dictionary<string, Operator> ServiceResponders = new Dictionary<string, Operator>();
        private Dictionary<string, Operator> ServiceCallers = new Dictionary<string, Operator>();

        public RosSocket(string url)
        {
            Protocol = new WebsocketProtocol(url);
            Protocol.OnMessage += (sender, e) => receivedOperation(sender, e);
            Protocol.Connect();
        }

        public void Close()
        {
       //     while (Publishers.Count > 0)
        //        Unadvertise(Publishers.First().Key);

         //   while (Subscribers.Count > 0)
          //      Unsubscribe(Subscribers.First().Key);

            while (ServiceResponders.Count > 0)
                UnadvertiseService(ServiceResponders.First().Key);

            Protocol.Close();
        }
        

        public string Advertise(string topic, string type) 
        {
            // todo delete previous entry if topic already exists
            string id = topic;
            Publishers.Add(id, new Publisher(topic));

            Send(new Adverisement(id, topic, type));
            return id;
        }
        
        public void Publish<T>(string id, T message) where T : Message
        {
            Send(new Publication<T>(id, Publishers[id].Topic, message));
        }

        public void Unadvertise(string id)
        {
            Send(new Unadverisement(id, Publishers[id].Topic));
            Publishers.Remove(id);
        }

        public string AdvertiseService<Tin,Tout>(string service, ServiceCallHandler<Tin,Tout> serviceCallHandler) where Tin : Message where Tout : Message
        {
            string id = service;
            ServiceResponder<Tin, Tout> ServiceResponder = new ServiceResponder<Tin, Tout>(service, serviceCallHandler);
            ServiceResponders.Add(id, ServiceResponder);

            Send(new ServiceAdvertisement(id, service, MessageTypes.Dictionary[typeof(Tin)] ));
            return id;
        }

        public void UnadvertiseService(string id)
        {
            Operator _operator;
            if (ServiceResponders.TryGetValue(id, out _operator))
                Send(new Unadverisement(id, _operator.Name));

            ServiceResponders.Remove(id);
        }

        public string Subscribe<T>(string topic, SubscriptionHandler<T> subscriptionHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") where T: Message
        {
            string id = topic;
            Subscribers.Add(topic, new Subscriber<T>(topic, subscriptionHandler));
            Send(new Subscription(id, topic, MessageTypes.Dictionary[typeof(T)], throttle_rate, queue_length, fragment_size, compression));
            return id;
        }

        public void Unsubscribe(string id)
        {
            Send(new Unsubscription(id, Subscribers[id].Name));
            Subscribers.Remove(id);
        }

        public string CallService<Tin,Tout>(string service, ServiceResponseHandler<Tout> serviceResponseHandler, Tin serviceArguments = null) where Tin : Message where Tout : Message
        {
            string id = service;
            ServiceCallers.Add(id, new ServiceCaller<Tout>(service, serviceResponseHandler));

            Send(new ServiceCall<Tin>(id, service, serviceArguments));
            return id;
        }

        private void receivedOperation(object sender, EventArgs e)
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

                        Type type = Subscribers[id].MessageType;
                        Message message =(Message)jObject.GetValue("msg").ToObject(Subscribers[id].MessageType); ;
                        Subscribers[id].Receive(message);
                        




                        Console.WriteLine("test");
                        //subscriber.s.Invoke(jObject.GetValue("msg").ToObject<Message>());

                        //    

                        // 

                        //  receivedPublish(jObject.ToObject<Publication>());
                        return;

                    }
                case "service_response":
                    {
                      //  receivedServiceResponse(jObject.ToObject<ServiceResponse>());
                        return;
                    }
                case "call_service":
                    {
                     //   receivedServiceCall(jObject.ToObject<ServiceCall>());
                        return;
                    }
            }
        }

        /*
             private void receivedPublish<T>(Publication<T> publication) where T: Message
             {
                 Subscriber<T> subscriber;
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
                 Send<ServiceResponse>(
                     new ServiceResponse(
                     serviceCall.id,
                     serviceCall.service,
                     JObject.FromObject(result),
                     isSuccess));
             }*/

        private void Send<T>(T operation) where T: Operation
        {
#if DEBUG
            Console.WriteLine("Sending:\n" +  JsonConvert.SerializeObject(operation, Formatting.Indented) + "\n");           
#endif
            Protocol.SendAsync(Serialize(operation), null);
        }

        public static byte[] Serialize<T>(T obj)
        {
            string json = JsonConvert.SerializeObject(obj);
            return Encoding.ASCII.GetBytes(json);
        }

        public static T Deserialize<T>(byte[] buffer)
        {
            string ascii = Encoding.ASCII.GetString(buffer, 0, buffer.Length);
            return JsonConvert.DeserializeObject<T>(ascii);
        }
    }
}
