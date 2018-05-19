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

using RosSharp.RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceResponseHandler<T>(T t) where T : Message;
    public delegate void SubscriptionHandler<T>(T t) where T : Message;
    public delegate bool ServiceCallHandler<Tin, Tout>(Tin tin, out Tout tout) where Tin : Message where Tout : Message;

    public abstract class Publisher
    {
        public abstract string Id { get; }
        public abstract string Topic { get; }

        public abstract Operation Publish(Message message);

        public Unadverisement Unadvertise()
        {
            return new Unadverisement(Id, Topic);
        }

    }
    public class Publisher<T> : Publisher where T : Message
    {
        public override string Id { get; }
        public override string Topic { get; }

        public Publisher(string id, string topic, out Advertisement advertisement)
        {
            Topic = topic;
            Id = id;
            advertisement = new Advertisement(Id, Topic, Message.GetRosName(typeof(T)) );
        }

        public override Operation Publish(Message message)
        {
            return new Publication<T>(Id, Topic, (T)message);
        }
    }

    public abstract class Subscriber
    {
        public abstract string Id { get; }
        public abstract string Topic { get; }

        public abstract void Receive(Message message);

        public Unsubscription Unsubscribe()
        {
            return new Unsubscription(Id, Topic);
        }
    }

    public class Subscriber<T> : Subscriber where T : Message
    {
        public override string Id { get; }
        public override string Topic { get; }

        public SubscriptionHandler<T> SubscriptionHandler { get; }

        public Subscriber(string id, string topic, SubscriptionHandler<T> subscriptionHandler, out Subscription subscription, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none")
        {
            Topic = topic;
            SubscriptionHandler = subscriptionHandler;
            subscription = new Subscription(Id, Topic, Message.GetRosName(typeof(T)), throttle_rate, queue_length, fragment_size, compression);
        }

        public override void Receive(Message message)
        {
            SubscriptionHandler.Invoke((T)message);
        }
    }

    public abstract class ServiceProvider
    {
        public abstract string Service { get; }

        public abstract Operation Respond(string id, Message args);

        public ServiceUnadvertisement UnadvertiseService()
        {
            return new ServiceUnadvertisement(Service);
        }
    }

    public class ServiceProvider<Tin, Tout> : ServiceProvider where Tin : Message where Tout : Message
    {
        public override string Service { get; }
        public ServiceCallHandler<Tin, Tout> ServiceCallHandler;
        public ServiceProvider(string service, ServiceCallHandler<Tin, Tout> serviceCallHandler, out ServiceAdvertisement serviceAdvertisement)
        {
            Service = service;
            ServiceCallHandler = serviceCallHandler;
            serviceAdvertisement = new ServiceAdvertisement(service, Message.GetRosName(typeof(Tin)));
        }

        public override Operation Respond(string id, Message args)
        {
            bool isSuccess = ServiceCallHandler.Invoke((Tin)args, out Tout result);
            return new ServiceResponse<Tout>(id, Service, result, isSuccess);
        }
    }

    public abstract class ServiceConsumer
    {
        public abstract string Id { get; }
        public abstract string Service { get; }
        public abstract void Consume(Message result);
    }

    public class ServiceConsumer<Tin, Tout> : ServiceConsumer where Tin : Message where Tout : Message
    {
        public override string Id { get; }
        public override string Service { get; }
        public ServiceResponseHandler<Tout> ServiceResponseHandler;

        public ServiceConsumer(string id, string service, ServiceResponseHandler<Tout> serviceResponseHandler, out Operation serviceCall, Tin serviceArguments = null)
        {
            Id = id;
            Service = service;
            ServiceResponseHandler = serviceResponseHandler;
            serviceCall = new ServiceCall<Tin>(id, service, serviceArguments);
        }
        public override void Consume(Message result)
        {
            ServiceResponseHandler.Invoke((Tout)result);
        }
    }
}
