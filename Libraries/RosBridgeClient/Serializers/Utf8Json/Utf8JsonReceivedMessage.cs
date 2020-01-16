/*
© University of Kent, 2019
Author: Odysseas Doumas <od79@kent.ac.uk>
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
