using RosSharp.RosBridgeClient.Messages;
using System;

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceResponseHandler<T>(T t) where T : Message;
    public delegate void SubscriptionHandler<T>(T t) where T : Message;
    public delegate bool ServiceCallHandler<Tin, Tout>(Tin tin, out Tout tout) where Tin : Message where Tout : Message;

    public abstract class Operator
    {
        public abstract string Name { get; }
        public virtual Type MessageType { get; }
        public abstract void Receive(Message message);
    }

    public class Publisher : Operator
    {
        public override string Name { get { return Topic; } }
        public string Topic;
        public Publisher(string topic)
        {
            Topic = topic;
        }
        public override void Receive(Message message) { }
    }

    public class ServiceResponder<Tin, Tout> : Operator where Tin : Message where Tout : Message
    {
        public override string Name { get { return Service; } }
        public string Service;
        public ServiceCallHandler<Tin, Tout> ServiceCallHandler;
        public ServiceResponder(string service, ServiceCallHandler<Tin, Tout> serviceCallHandler)
        {
            Service = service;
            ServiceCallHandler = serviceCallHandler;
        }
        public override void Receive(Message message) { }
    }


    public class Subscriber<T> : Operator where T : Message
    {
        public override string Name { get { return Topic; } }
        public override Type MessageType { get { return typeof(T); } }
        public string Topic;
        public SubscriptionHandler<T> SubscriptionHandler;
        public Subscriber(string topic, SubscriptionHandler<T> subscriptionHandler)
        {
            Topic = topic;
            SubscriptionHandler = subscriptionHandler;
        }
        public override void Receive(Message message)
        {
            SubscriptionHandler.Invoke(message as T);
        }
    }

    public class ServiceCaller<T> : Operator where T : Message
    {
        public override string Name { get { return Service; } }
        public string Service;
        public Type ServiceResponseType;
        public ServiceResponseHandler<T> ServiceResponseHandler;
        public ServiceCaller(string service, ServiceResponseHandler<T> serviceResponseHandler)
        {
            Service = service;
            ServiceResponseHandler = serviceResponseHandler;
            ServiceResponseType = typeof(T);
        }
        public override void Receive(Message message) { }
    }
}
