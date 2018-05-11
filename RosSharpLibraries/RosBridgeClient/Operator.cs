using System;
using Newtonsoft.Json.Linq;

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceHandler(object obj);
    public delegate void MessageHandler(Message message);
    public delegate bool ServiceCallHandler(JObject arguments, out JObject result);

    public class Operator
    {
    }

    public class Publisher : Operator
    {
        internal string Topic;
        internal Publisher(string topic)
        {
            Topic = topic;
        }
    }

    public class ServiceResponder: Operator
    {
        internal string Service;
        internal string ArgumentType;
        internal ServiceCallHandler ServiceCallHandler;
        internal ServiceResponder(string service, string argumentType, ServiceCallHandler serviceCallHandler)
        {
            Service = service;
            ArgumentType = argumentType;
            ServiceCallHandler = serviceCallHandler;
        }
    }

    public class Subscriber : Operator
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
    public class ServiceCaller : Operator
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
}
