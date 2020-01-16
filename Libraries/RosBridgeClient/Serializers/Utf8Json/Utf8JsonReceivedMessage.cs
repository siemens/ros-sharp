using System;
using System.Runtime.Serialization;
using Utf8Json;

namespace RosSharp.RosBridgeClient.Serializers
{
    class Utf8JsonReceivedMessage : IReceivedMessage
    {
        public byte[] buffer;
        private readonly int offset = 0;

        public Utf8JsonReceivedMessage(string op, string topic, string service, string id, int offset)
        {
            this.Op = op;
            this.Id = id;
            this.Topic = topic;
            this.Service = service;
            this.offset = offset;
        }

        public string Op { get; private set; }

        public string Id { get; private set; }

        public string Topic { get; private set; }

        public string Service { get; private set; }

        public T GetArgs<T>() where T : Message
        {
            return getObject<T>("call_service");
        }

        public T GetMessage<T>() where T : Message
        {
            return getObject<T>("publish");
        }

        public T GetValues<T>() where T : Message
        {
            return getObject<T>("service_response");
        }

        private T getObject<T>(string id)
        {
            return (Op == id) ? JsonSerializer.Deserialize<T>(buffer, offset, RosSharpDefaultResolver.Instance) : default(T);

        }
    }
}
