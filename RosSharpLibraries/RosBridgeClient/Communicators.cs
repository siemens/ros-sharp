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

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceResponseHandler<T>(T t) where T : Message;
    public delegate void SubscriptionHandler<T>(T t) where T : Message;
    public delegate bool ServiceCallHandler<Tin, Tout>(Tin tin, out Tout tout) where Tin : Message where Tout : Message;

    internal abstract class Publisher
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
            Topic = topic;
            Id = id;
            advertisement = new Advertisement(Id, Topic, Message.GetRosName(typeof(T)) );
        }

        internal override Communication Publish(Message message)
        {
            return new Publication<T>(Id, Topic, (T)message);
        }
    }

    internal abstract class Subscriber
    {
        internal abstract string Id { get; }
        internal abstract string Topic { get; }

        internal abstract void Receive(Message message);

        internal Unsubscription Unsubscribe()
        {
            return new Unsubscription(Id, Topic);
        }
    }

    internal class Subscriber<T> : Subscriber where T : Message
    {
        internal override string Id { get; }
        internal override string Topic { get; }

        internal SubscriptionHandler<T> SubscriptionHandler { get; }

        internal Subscriber(string id, string topic, SubscriptionHandler<T> subscriptionHandler, out Subscription subscription, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            Topic = topic;
            SubscriptionHandler = subscriptionHandler;
            subscription = new Subscription(Id, Topic, Message.GetRosName(typeof(T)), throttle_rate, queue_length, fragment_size, compression);
        }

        internal override void Receive(Message message)
        {
            SubscriptionHandler.Invoke((T)message);
        }
    }

    internal abstract class ServiceProvider
    {
        internal abstract string Service { get; }

        internal abstract Communication Respond(string id, Message args);

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
            serviceAdvertisement = new ServiceAdvertisement(service, Message.GetRosName(typeof(Tin)));
        }

        internal override Communication Respond(string id, Message args)
        {
            bool isSuccess = ServiceCallHandler.Invoke((Tin)args, out Tout result);
            return new ServiceResponse<Tout>(id, Service, result, isSuccess);
        }
    }

    internal abstract class ServiceConsumer
    {
        internal abstract string Id { get; }
        internal abstract string Service { get; }
        internal abstract void Consume(Message result);
    }

    internal class ServiceConsumer<Tin, Tout> : ServiceConsumer where Tin : Message where Tout : Message
    {
        internal override string Id { get; }
        internal override string Service { get; }
        internal ServiceResponseHandler<Tout> ServiceResponseHandler;

        internal ServiceConsumer(string id, string service, ServiceResponseHandler<Tout> serviceResponseHandler, out Communication serviceCall, Tin serviceArguments = null)
        {
            Id = id;
            Service = service;
            ServiceResponseHandler = serviceResponseHandler;
            serviceCall = new ServiceCall<Tin>(id, service, serviceArguments);
        }
        internal override void Consume(Message result)
        {
            ServiceResponseHandler.Invoke((Tout)result);
        }
    }
}
