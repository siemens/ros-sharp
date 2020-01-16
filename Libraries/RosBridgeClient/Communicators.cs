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

// Added UTF8Json (de-)seriliazation option
// University of Kent, 2019, Odysseas Doumas (od79@kent.ac.uk)

using RosSharp.RosBridgeClient.Serializers;
using System;

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceResponseHandler<T>(T t) where T : Message;
    public delegate void SubscriptionHandler<T>(T t) where T : Message;
    public delegate bool ServiceCallHandler<Tin, Tout>(Tin tin, out Tout tout) where Tin : Message where Tout : Message;

    internal abstract class Communicator
    {
        public static string GetRosName<T>() where T : Message
        {
            return (string)typeof(T).GetField("RosMessageName").GetRawConstantValue();
        }
    }
    internal abstract class Publisher : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }

        internal abstract Communication Publish(Message message);

        internal abstract void Return(Communication comm);

        internal Unadvertisement Unadvertise()
        {
            return new Unadvertisement(Id, Topic);
        }
    }

    internal class Publisher<T> : Publisher where T : Message
    {
        private static ObjectPool<Publication<T>> pool = new ObjectPool<Publication<T>>(() => new Publication<T>("", "", null));

        internal override void Return(Communication comm)
        {
            pool.PutObject((Publication<T>)comm);
        }

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
            Publication<T> publication = pool.GetObject();
            publication.id = Id;
            publication.topic = Topic;
            publication.msg = (T)message;
            return publication;
        }
    }

    internal abstract class Subscriber : Communicator
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }
        internal abstract Type TopicType { get; }

        internal abstract void Receive(IReceivedMessage handle);

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

        internal override void Receive(IReceivedMessage handle)
        {
            SubscriptionHandler.Invoke(handle.GetMessage<T>());
        }
    }

    internal abstract class ServiceProvider : Communicator
    {
        internal abstract string Service { get; }

        internal abstract Communication Respond(string id, IReceivedMessage handle);

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

        internal override Communication Respond(string id, IReceivedMessage handle)
        {
            bool isSuccess = ServiceCallHandler.Invoke(handle.GetArgs<Tin>(), out Tout result);
            return new ServiceResponse<Tout>(id, Service, result, isSuccess);
        }
    }

    internal abstract class ServiceConsumer
    {
        internal abstract string Id { get; }
        internal abstract string Service { get; }
        internal abstract void Consume(IReceivedMessage handle);
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
        internal override void Consume(IReceivedMessage handle)
        {
            ServiceResponseHandler.Invoke(handle.GetValues<Tout>());
        }
    }
}
