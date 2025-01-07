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
using System.Threading;
using RosSharp.RosBridgeClient.Protocols;

namespace RosSharp.RosBridgeClient
{
    public class RosSocket
    {
        public IProtocol protocol;
        public enum SerializerEnum { Microsoft, Newtonsoft_JSON }

        public SerializerEnum SerializerType;
        
        private readonly Dictionary<SerializerEnum, ISerializer> serializerDictionary = new Dictionary<SerializerEnum, ISerializer>()
        {
            { SerializerEnum.Microsoft, new MicrosoftSerializer()},
            { SerializerEnum.Newtonsoft_JSON, new NewtonsoftJsonSerializer()}
        };
        private Dictionary<string, Publisher> Publishers = new Dictionary<string, Publisher>();
        private Dictionary<string, Subscriber> Subscribers = new Dictionary<string, Subscriber>();
        private Dictionary<string, ServiceProvider> ServiceProviders = new Dictionary<string, ServiceProvider>();
        private Dictionary<string, ServiceConsumer> ServiceConsumers = new Dictionary<string, ServiceConsumer>();

#if ROS2
        private Dictionary<string, ActionProvider> ActionProviders = new Dictionary<string, ActionProvider>(); 
        private Dictionary<string, ActionConsumer> ActionConsumers = new Dictionary<string, ActionConsumer>();
#endif
        internal ISerializer Serializer;
        private object SubscriberLock = new object();

        public RosSocket(IProtocol protocol, SerializerEnum serializer = SerializerEnum.Microsoft)
        {
            this.protocol = protocol;

            if (serializerDictionary.TryGetValue(serializer, out Serializer))
            {
                SerializerType = serializer;
            }
            else
            {
                throw new ArgumentException("Invalid serializer type specified.");
            }

            this.protocol.OnReceive += (sender, e) => Receive(sender, e);
            this.protocol.Connect();
        }

        public void Close(int millisecondsWait = 0) // TODO Actions
        {
#if ROS2
            bool isAnyCommunicatorActive = Publishers.Count > 0 || Subscribers.Count > 0 || ServiceProviders.Count > 0 || ActionProviders.Count > 0 || ActionConsumers.Count > 0;
#else
            bool isAnyCommunicatorActive = Publishers.Count > 0 || Subscribers.Count > 0 || ServiceProviders.Count > 0;
#endif
            while (Publishers.Count > 0)
                Unadvertise(Publishers.First().Key);

            while (Subscribers.Count > 0)
                Unsubscribe(Subscribers.First().Key);

            while (ServiceProviders.Count > 0)
                UnadvertiseService(ServiceProviders.First().Key);

#if ROS2
            while (ActionProviders.Count > 0)  
                UnadvertiseAction(ActionProviders.First().Key);
#endif

            // Service/Action consumers do not stay on. So nothing to unsubscribe/unadvertise

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

        public string Subscribe<T>(string topic, SubscriptionHandler<T> subscriptionHandler, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none", bool ensureThreadSafety = false) where T : Message
        {
            string id;
            lock (SubscriberLock)
            {
                id = GetUnusedCounterID(Subscribers, topic);
                Subscription subscription;

                var subscriber = new Subscriber<T>(id, topic, subscriptionHandler, out subscription, throttle_rate, queue_length, fragment_size, compression)
                {
                    DoEnsureThreadSafety = ensureThreadSafety
                };

                Subscribers.Add(id, subscriber);
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

#if ROS2
        #region ActionProvider

        public string AdvertiseAction<TActionGoal, TActionFeedback, TActionResult>(
            string action,
            SendActionGoalHandler<TActionGoal> sendActionGoalHandler,
            CancelActionGoalHandler cancelActionGoalHandler)
            where TActionGoal : Message
            where TActionFeedback : Message
            where TActionResult : Message
        {
            string id = action;
            if (ActionProviders.ContainsKey(id))
                UnadvertiseAction(id);

            ActionAdvertisement actionAdvertisement;
            ActionProviders.Add(id, new ActionProvider<TActionGoal>(
                action,
                sendActionGoalHandler,
                cancelActionGoalHandler,
                out actionAdvertisement));
            Send(actionAdvertisement);

            return id;
        }

        public void RespondFeedback<TActionFeedback, TFeedback>(string id, TActionFeedback actionFeedback)
            where TActionFeedback : ActionFeedback<TFeedback>
            where TFeedback : Message
        {
            Send(ActionProviders[id].RespondFeedback<TActionFeedback, TFeedback>(actionFeedback));
        }
        public void RespondResult<TActionResult, TResult>(string id, TActionResult ActionResult)
            where TActionResult : ActionResult<TResult>
            where TResult : Message
        {
            Send(ActionProviders[id].RespondResult<TActionResult, TResult>(ActionResult));
        }
        public void UnadvertiseAction(string id)
        {
            Send(ActionProviders[id].UnadvertiseAction());
            ActionProviders.Remove(id);
        }
         
        #endregion
        #region ActionConsumers

        public string CancelActionGoalRequest<TActionResult>(
            string frameId,
            string action,
            ActionCancelResponseHandler<TActionResult> actionCancelResponseHandler)
            where TActionResult : Message
        {
            string id = GetUnusedCounterID(ActionConsumers, action);
            ActionConsumers.Add(id , new ActionConsumer<TActionResult, Message>(
                id,
                action,
                actionCancelResponseHandler: actionCancelResponseHandler)
            );

            Send(ActionConsumers[id].CancelActionGoalRequest(frameId, action));
            return id;
        }

        public string SendActionGoalRequest<TActionGoal, TGoal, TActionFeedback, TActionResult>(
            TActionGoal actionGoal,
            ActionResultResponseHandler<TActionResult> actionResultResponseHandler,
            ActionFeedbackResponseHandler<TActionFeedback> actionFeedbackResponseHandler)
            where TActionGoal : ActionGoal<TGoal>
            where TGoal : Message
            where TActionResult : Message
            where TActionFeedback : Message
        {
            string id = GetUnusedCounterID(ActionConsumers, actionGoal.action);
            ActionConsumers.Add(id, new ActionConsumer<TActionResult, TActionFeedback>(
                id,
                actionGoal.action,
                actionResultResponseHandler: actionResultResponseHandler,
                actionFeedbackResponseHandler: actionFeedbackResponseHandler)
            );

            Send(ActionConsumers[id].SendActionGoalRequest<TActionGoal, TGoal>(actionGoal));
            return id;
        }
        #endregion

#endif
        private void Send<T>(T communication) where T : Communication
        {
            //var serialized = Serializer.Serialize(communication);
            //DeserializedObject deserializedObject = Serializer.Deserialize(serialized);
            //Console.WriteLine("Complete outgoing message: " + deserializedObject.GetAll());
            protocol.Send(Serializer.Serialize<T>(communication));
            return;
        }

        private void Receive(object sender, EventArgs e)
        {
            byte[] buffer = ((MessageEventArgs)e).RawData;
            DeserializedObject jsonElement = Serializer.Deserialize(buffer);

            switch (jsonElement.GetProperty("op"))            
            {
                case "publish":
                    {
                        string topic = jsonElement.GetProperty("topic");
                        string msg = jsonElement.GetProperty("msg");
                        foreach (Subscriber subscriber in SubscribersOf(topic))
                            subscriber.Receive(msg, Serializer);
                        return;
                    }
                case "service_response":
                    {
                        string id = jsonElement.GetProperty("id");
                        string values = jsonElement.GetProperty("values");
                        ServiceConsumers[id].Consume(values, Serializer);
                        return;
                    }
                case "call_service":
                    {
                        string id = jsonElement.GetProperty("id");
                        string service = jsonElement.GetProperty("service");
                        string args = jsonElement.GetProperty("args");
                        Send(ServiceProviders[service].Respond(id, args, Serializer));
                        return;
                    }
#if ROS2
                // Provider side
                case "send_action_goal": 
                    {
                        string action = jsonElement.GetProperty("action");

                        //Console.WriteLine("Complete incoming goal message: " + jsonElement.GetAll());
                        ActionProviders[action].ListenSendGoalAction(jsonElement.GetAll(), Serializer);
                        return;
                    }
                // Provider side
                case "cancel_action_goal":
                    {
                        string frameId = jsonElement.GetProperty("id");
                        string action = jsonElement.GetProperty("action");

                        //Console.WriteLine("Complete incoming cancel message: " + jsonElement.GetAll());
                        ActionProviders[action].ListenCancelGoalAction(frameId, action, Serializer);
                        return;
                    }
                // Consumer side
                case "action_feedback":
                    {
                        string id = jsonElement.GetProperty("id");

                        //Console.WriteLine("Complete server response for feedback: " + jsonElement.GetAll());
                        ActionConsumers[id].ConsumeFeedbackResponse(jsonElement.GetAll(), Serializer);
                        return;
                    }
                // Consumer side
                case "action_result":
                    {
                        string id = jsonElement.GetProperty("id");

                        //Console.WriteLine("Complete server response for result: " + jsonElement.GetAll());
                        ActionConsumers[id].ConsumeResultResponse(jsonElement.GetAll(), Serializer);
                        return;
                    }
#endif
            }
        }

        private List<Subscriber> SubscribersOf(string topic)
        {
            return Subscribers.Where(pair => pair.Key.StartsWith(topic + ":")).Select(pair => pair.Value).ToList();
        }

        private static string GetUnusedCounterID<T>(Dictionary<string, T> dictionary, string name)
        {
            int counter = 0;
            string id;
            do
            {
                id = name + ":" + counter++;
            }
            while (dictionary.ContainsKey(id));
            return id;
        }
    }
}
