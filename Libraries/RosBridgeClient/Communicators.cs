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

using Newtonsoft.Json.Linq;
using System;
using System.Reflection;

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceResponseHandler<T>(T t) where T : Message;
    public delegate void SubscriptionHandler<T>(T t) where T : Message;
    public delegate bool ServiceCallHandler<Tin, Tout>(Tin tin, out Tout tout) where Tin : Message where Tout : Message;

    internal abstract class Communicator
    {
        public static string GetRosName<T>() where T : Message
        {
#if !WINDOWS_UWP
            return (string)typeof(T).GetField("RosMessageName").GetRawConstantValue();
#else
            return (string)typeof(T).GetTypeInfo().GetDeclaredField("RosMessageName").GetValue(null);
#endif
        }
    }
    internal abstract class Publisher : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }

        internal abstract Communication Publish(Message message);

        internal Unadverisement Unadvertise()
        {
            return new Unadverisement(Id, Topic);
        }
    }

    internal class Publisher<T> : Publisher where T : Message
    {
        internal override string Id { get; }
        internal override string Topic { get; }

        internal Publisher(string id, string topic, out Advertisement advertisement)
        {
            Id = id;
            Topic = topic;
            advertisement = new Advertisement(Id, Topic, GetRosName<T>());
        }

        internal override Communication Publish(Message message)
        {
            return new Publication<T>(Id, Topic, (T)message);
        }
    }

    internal abstract class Subscriber : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }
        internal abstract Type TopicType { get; }

        internal abstract void Receive(JToken message);

        internal Unsubscription Unsubscribe()
        {
            return new Unsubscription(Id, Topic);
        }
    }

    internal class Subscriber<T> : Subscriber where T : Message
    {
        internal override string Id { get; }
        internal override string Topic { get; }
        internal override Type TopicType { get { return typeof(T); } }

        internal SubscriptionHandler<T> SubscriptionHandler { get; }

        internal Subscriber(string id, string topic, SubscriptionHandler<T> subscriptionHandler, out Subscription subscription, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            Id = id;
            Topic = topic;
            SubscriptionHandler = subscriptionHandler;
            subscription = new Subscription(id, Topic, GetRosName<T>(), throttle_rate, queue_length, fragment_size, compression);
        }

        internal override void Receive(JToken message)
        {
            SubscriptionHandler.Invoke(message.ToObject<T>());
        }
    }

    internal abstract class ServiceProvider : Communicator
    {
        internal abstract string Service { get; }

        internal abstract Communication Respond(string id, JToken args = null);

        internal ServiceUnadvertisement UnadvertiseService()
        {
            return new ServiceUnadvertisement(Service);
        }
    }

    internal class ServiceProvider<Tin, Tout> : ServiceProvider where Tin : Message where Tout : Message
    {
        internal override string Service { get; }
        internal ServiceCallHandler<Tin, Tout> ServiceCallHandler;
        internal ServiceProvider(string service, ServiceCallHandler<Tin, Tout> serviceCallHandler, out ServiceAdvertisement serviceAdvertisement)
        {
            Service = service;
            ServiceCallHandler = serviceCallHandler;
            serviceAdvertisement = new ServiceAdvertisement(service, GetRosName<Tin>());
        }

        internal override Communication Respond(string id, JToken args = null)
        {
            Tout result;
            bool isSuccess = ServiceCallHandler.Invoke(args.ToObject<Tin>(), out result);
            return new ServiceResponse<Tout>(id, Service, result, isSuccess);
        }
    }

    internal abstract class ServiceConsumer
    {
        internal abstract string Id { get; }
        internal abstract string Service { get; }
        internal abstract void Consume(JToken result);
    }

    internal class ServiceConsumer<Tin, Tout> : ServiceConsumer where Tin : Message where Tout : Message
    {
        internal override string Id { get; }
        internal override string Service { get; }
        internal ServiceResponseHandler<Tout> ServiceResponseHandler;

        internal ServiceConsumer(string id, string service, ServiceResponseHandler<Tout> serviceResponseHandler, out Communication serviceCall, Tin serviceArguments)
        {
            Id = id;
            Service = service;
            ServiceResponseHandler = serviceResponseHandler;
            serviceCall = new ServiceCall<Tin>(id, service, serviceArguments);
        }
        internal override void Consume(JToken result)
        {
            ServiceResponseHandler.Invoke(result.ToObject<Tout>());
        }
    }
}
