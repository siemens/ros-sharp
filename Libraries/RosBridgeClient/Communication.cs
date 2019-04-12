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
        public string op; // required
        public string id; // optional

        internal Communication(string id = null)
        {
            this.id = id;
        }
    }

    internal class Advertisement : Communication
    {
        public string topic; // required
        public string type; // required

        internal Advertisement(string id, string topic, string type) : base(id)
        {
            this.op = "advertise";
            this.topic = topic;
            this.type = type;
        }
    }

    internal class Unadvertisement : Communication
    {
        public string topic; // required

        internal Unadvertisement(string id, string topic) : base(id)
        {
            this.op = "unadvertise";
            this.topic = topic;
        }
    }

    internal class Publication<T> : Communication where T: Message
    {
        public string topic; // required
        public T msg; // required

        internal Publication(string id, string topic, T msg) : base(id)
        {
            this.op = "publish";
            this.topic = topic;
            this.msg = msg;
        }
    }

    internal class Subscription : Communication
    {
        public string topic; // required
        public string type; // optional
        public int throttle_rate; // optional
        public int queue_length; // optional
        public int fragment_size; // optional
        public string compression; // optional

        internal Subscription(string id, string topic, string type, int throttle_rate = 0, int queue_length = 1, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.op = "subscribe";
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
        public string topic; // required

        internal Unsubscription(string id, string topic) : base(id)
        {
            this.op = "unsubscribe";
            this.topic = topic;
        }
    }

    internal class ServiceCall<T> : Communication where T : Message
    {
        public string service; // required
        public T args; // optional
        public int fragment_size; // optional
        public string compression; // optional

        public ServiceCall(string id, string service, T args, int fragment_size = int.MaxValue, string compression = "none") : base(id)
        {
            this.op = "call_service";
            this.service = service;
            this.args = args;
            this.fragment_size = fragment_size;
            this.compression = compression;
        }
    }

    internal class ServiceResponse<T> : Communication where T : Message
    {
        public string service; // required
        public T values; // optional
        public bool result; // required

        internal ServiceResponse(string id, string service, T values, bool Result) : base(id)
        {
            this.op = "service_response";
            this.service = service;
            this.values = values;
            result = Result;
        }
    }
    internal class ServiceAdvertisement : Communication
    {
        public string type; // required
        public string service; // required

        internal ServiceAdvertisement(string service, string type) 
        {
            this.op = "advertise_service";
            this.service = service;
            this.type = type;
        }
    }
    internal class ServiceUnadvertisement : Communication
    {
        public string service; // required

        internal ServiceUnadvertisement(string Service)
        {
            this.op = "unadvertise_service";
            service = Service;
        }
    }
}
