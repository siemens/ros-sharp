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
    public class Operation
    {
        public virtual string op { get { return "undefined"; } } // required
        public string id; // optional

        public Operation(string id = null)
        {
            this.id = id;
        }
    }

    public class Adverisement : Operation
    {
        public override string op { get { return "advertise"; } } // required
        public string topic; // required
        public string type; // required

        public Adverisement(string id, string topic, string type) : base(id)
        {
            this.topic = topic;
            this.type = type;
        }
    }

    public class Unadverisement : Operation
    {
        public override string op { get { return "unadvertise"; } } // required
        public string topic; // required

        public Unadverisement(string id, string topic) : base(id)
        {
            this.topic = topic;
        }
    }

    public class Publication : Operation
    {
        public override string op { get { return "publish"; } } // required
        public string topic; // required
        public Message msg; // required

        public Publication(string id, string topic, Message msg) : base(id)
        {
            this.topic = topic;
            this.msg = msg;
        }
    }

    public class Subscription : Operation
    {
        public override string op { get { return "subscribe"; } } // required
        public string topic; // required
        public string type; // optional
        public int throttle_rate; // optional
        public int queue_length; // optional
        public int fragment_size; // optional
        public string compression; // optional

        public Subscription(string id, string topic, string type, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.topic = topic;
            this.type = type;
            this.throttle_rate = throttle_rate;
            this.queue_length = queue_length;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    public class Unsubscription : Operation
    {
        public override string op { get { return "unsubscribe"; } } // required
        public string topic; // required

        public Unsubscription(string id, string topic) : base(id)
        {
            this.topic = topic;
        }
    }

    public class ServiceCall : Operation
    {
        public override string op { get { return "call_service"; } } // required
        public string service; // required
        public object args; // optional
        public int fragment_size; // optional
        public string compression; // optional

        public ServiceCall(string id, string service, object args = null, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.service = service;
            this.args = args;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    public class ServiceResponse : Operation
    {
        public override string op { get { return "service_response"; } } // required
        public string service; // required
        public object values; // optional
        public bool result;

        public ServiceResponse(string id, string service, object values, bool Result) : base(id)
        {
            this.service = service;
            this.values = values;
            result = Result;
        }
    }
    public class ServiceAdvertisement : Operation
    {
        public override string op { get { return "advertise_service"; } } // required
        public string type; // required
        public string service; // required


        public ServiceAdvertisement(string id, string service, string type) : base(id)
        {
            this.service = service;
            this.type = type;
        }
    }
    public class ServiceUnadvertisement : Operation
    {
        public override string op { get { return "unadvertise_service"; } } // required
        public string service; // required

        public ServiceUnadvertisement(string Id, string Service) : base(Id)
        {
            service = Service;
        }
    }
}
