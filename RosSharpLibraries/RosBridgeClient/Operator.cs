using System;
using Newtonsoft.Json.Linq;
using RosSharp.RosBridgeClient.Messages;

namespace RosSharp.RosBridgeClient
{
    public delegate void ServiceHandler(Message message);
    public delegate void MessageHandler(Message message);
    public delegate bool ServiceCallHandler(Message arguments, out Message result);

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
        internal Type ArgumentType;
        internal ServiceCallHandler ServiceCallHandler;
        internal ServiceResponder(string service, Type argumentType, ServiceCallHandler serviceCallHandler)
        {
            Service = service;
            ArgumentType = argumentType;
            ServiceCallHandler = serviceCallHandler;
        }
    }

    public class Subscriber : Operator
    {
        internal string Topic;
        internal Type MessageType;
        internal MessageHandler MessageHandler;
        internal Subscriber(string topic, Type messageType, MessageHandler messageHandler)
        {
            Topic = topic;
            MessageType = messageType;
            MessageHandler = messageHandler;
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
