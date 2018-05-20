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
    internal abstract class Communication
    {
        internal abstract string op { get; } // required
        internal virtual string id { get; } // optional

        internal Communication(string id = null)
        {
            this.id = id;
        }
    }

    internal class Advertisement : Communication
    {
        internal override string op { get { return "advertise"; } } // required
        internal string topic; // required
        internal string type; // required

        internal Advertisement(string id, string topic, string type) : base(id)
        {
            this.topic = topic;
            this.type = type;
        }
    }

    internal class Unadverisement : Communication
    {
        internal override string op { get { return "unadvertise"; } } // required
        internal string topic; // required

        internal Unadverisement(string id, string topic) : base(id)
        {
            this.topic = topic;
        }
    }

    internal class Publication<T> : Communication where T: Message
    {
        internal override string op { get { return "publish"; } } // required
        internal string topic; // required
        internal T msg; // required

        internal Publication(string id, string topic, T msg) : base(id)
        {
            this.topic = topic;
            this.msg = msg;
        }
    }

    internal class Subscription : Communication
    {
        internal override string op { get { return "subscribe"; } } // required
        internal string topic; // required
        internal string type; // optional
        internal int throttle_rate; // optional
        internal int queue_length; // optional
        internal int fragment_size; // optional
        internal string compression; // optional

        internal Subscription(string id, string topic, string type, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.topic = topic;
            this.type = type;
            this.throttle_rate = throttle_rate;
            this.queue_length = queue_length;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    internal class Unsubscription : Communication
    {
        internal override string op { get { return "unsubscribe"; } } // required
        internal string topic; // required

        internal Unsubscription(string id, string topic) : base(id)
        {
            this.topic = topic;
        }
    }

    internal class ServiceCall<T> : Communication where T : Message
    {
        internal override string op { get { return "call_service"; } } // required
        internal string service; // required
        internal T args; // optional
        internal int fragment_size; // optional
        internal string compression; // optional

        public ServiceCall(string id, string service, T args = null, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.service = service;
            this.args = args;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    internal class ServiceResponse<T> : Communication where T : Message
    {
        internal override string op { get { return "service_response"; } } // required
        internal string service; // required
        internal T values; // optional
        internal bool result; // required

        internal ServiceResponse(string id, string service, T values, bool Result) : base(id)
        {
            this.service = service;
            this.values = values;
            result = Result;
        }
    }
    internal class ServiceAdvertisement : Communication
    {
        internal override string op { get { return "advertise_service"; } } // required
        internal string type; // required
        internal string service; // required

        internal ServiceAdvertisement(string service, string type) 
        {
            this.service = service;
            this.type = type;
        }
    }
    internal class ServiceUnadvertisement : Communication
    {
        internal override string op { get { return "unadvertise_service"; } } // required
        internal string service; // required

        internal ServiceUnadvertisement(string Service)
        { 
            service = Service;
        }
    }
}
